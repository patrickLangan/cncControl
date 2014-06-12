#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glob.h>

#include <math.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include <time.h>

#define PRU_NUM 0
#define OFFSET_SHAREDRAM 2048

#define exitFunction(a) \
	cleanup (a); \
	return a;

/*
 * Get rid of exitFunction macro
 * Motors are slow to respond
 * Jumps in position between lines and arcs
 * Only run when a new sensor reading comes in
 * Replace pwm with pru program
 * Clean up this abomination
 */

struct axisInfo xAxis;
struct axisInfo yAxis;
struct axisInfo zAxis;

struct axisInfo {
	FILE *run;
	FILE *duty;
	FILE *period;
	FILE *direction;
};

struct vector {
        float	x,
                y,
                z;
};

struct arcInfo {
        struct vector	toSurf,
        		center,
        		cross;
        float	radius,
		angle,
		accel,
		time;
};

struct lineInfo {
        struct vector	velocity,
        		start,
        		end;
        float time;
};

struct rampInfo {
        struct vector accel;
        float time;
};

struct vector sensorInit = {0.0, 0.0, 0.0};

static unsigned int *sharedMem_int;

struct vector addVec (struct vector in1, struct vector in2)
{
        return (struct vector){in1.x + in2.x, in1.y + in2.y, in1.z + in2.z};
}

struct vector subVec (struct vector in1, struct vector in2)
{
        return (struct vector){in1.x - in2.x, in1.y - in2.y, in1.z - in2.z};
}

struct vector multVec (struct vector vec, float scaler)
{
        return (struct vector){vec.x * scaler, vec.y * scaler, vec.z * scaler};
}

float magnitude (struct vector input)
{
        return fabs (sqrt (pow (input.x, 2) + pow (input.y, 2) + pow (input.z, 2)));
}

int normalize (struct vector *input)
{
	float mag;
	
	mag = magnitude (*input);

	input->x /= mag;
	input->y /= mag;
	input->z /= mag;

	return 0;
}

struct vector getArcPos (struct arcInfo arc, float angle)
{
        float  cosAngle,
                sinAngle;

        cosAngle = cos (angle);
        sinAngle = sin (angle);

        return (struct vector){
                (cosAngle * arc.toSurf.x) + (sinAngle * arc.cross.x) + arc.center.x,
                (cosAngle * arc.toSurf.y) + (sinAngle * arc.cross.y) + arc.center.y,
                (cosAngle * arc.toSurf.z) + (sinAngle * arc.cross.z) + arc.center.z
        };
}

float gettimefromfunction (struct timeval startTime)
{
	struct timeval curTime;

	gettimeofday (&curTime, NULL);

	return ((float)(curTime.tv_sec - startTime.tv_sec) * 1e3) + ((float)(curTime.tv_usec - startTime.tv_usec) * 1e-3);
}

void startupAxis (struct axisInfo *axis, unsigned int number, char letter)
{
	FILE *pipe;
	char bashCommand[22];
	char runPath[37];
	char dutyPath[38];
	char periodPath[40];
	char directionPath[29];

	sprintf (bashCommand, "./startupAxis.sh %d %c", number, letter);
	pipe = popen (bashCommand, "r");

	fgets (runPath, 255, pipe);
	runPath[36] = '\0';
	axis->run = fopen (runPath, "w");

	fgets (dutyPath, 255, pipe);
	dutyPath[37] = '\0';
	axis->duty = fopen (dutyPath, "w");

	fgets (periodPath, 255, pipe);
	periodPath[39] = '\0';
	axis->period = fopen (periodPath, "w");

	fgets (directionPath, 255, pipe);
	directionPath[28] = '\0';
	axis->direction = fopen (directionPath, "w");

	pclose (pipe);
}

unsigned int shutdownAxis (struct axisInfo *axis)
{
	if (axis->run) {
		fprintf (axis->run, "0");
		fclose (axis->run);
	}

	if (axis->duty) {
		fprintf (axis->duty, "0");
		fclose (axis->duty);
	}

	if (axis->period)
		fclose (axis->period);

	if (axis->direction)
		fclose (axis->direction);

	return 0;
}

void waitEvent (void)
{
	prussdrv_pru_wait_event (PRU_EVTOUT_0);
	prussdrv_pru_clear_event (PRU0_ARM_INTERRUPT);
	prussdrv_pru_wait_event (PRU_EVTOUT_0);
	prussdrv_pru_clear_event (PRU0_ARM_INTERRUPT);
}

void readSensors (float *x, float *y, float *z)
{
	int resultX = sharedMem_int[OFFSET_SHAREDRAM + 0];
	int resultY = sharedMem_int[OFFSET_SHAREDRAM + 1];
	int resultZ = sharedMem_int[OFFSET_SHAREDRAM + 2];

	if (resultX >= 32768) {
		resultX |= 4294901760LL;
	}
	if (resultY >= 32768) {
		resultY |= 4294901760LL;
	}
	if (resultZ >= 32768) {
		resultZ |= 4294901760LL;
	}

	*x = 0.0003904191149 * ((float)resultX);
	*y = 0.0003904191149 * ((float)resultY);
	*z = 0.0003904191149 * ((float)resultZ);
}

unsigned int setVelocity (struct axisInfo *axis, float velocity)
{
	static unsigned int direction = 1;
	float speed;
	unsigned int period;
	unsigned int duty;

	if (velocity < 0) {
		speed = fabs (velocity);
		if (direction != 0) {
			direction = 0;
			fprintf (axis->direction, "0");
			fflush (axis->direction);
		}
	} else {
		speed = velocity;
		if (direction != 1) {
			direction = 1;
			fprintf (axis->direction, "1");
			fflush (axis->direction);
		}
	}

	period = (int)(500000.0 / speed);
	duty = (int)(250000.0 / speed);

	fprintf (axis->duty, "0");
	fflush (axis->duty);
	fprintf (axis->period, "%d", period);
	fflush (axis->period);
	fprintf (axis->duty, "%d", duty);
	fflush (axis->duty);

	return 0;
}

void PID (struct vector setPoint, struct vector velocity, float time)
{
	static float lastTime = 0.0;
	static float error = 0.0;
	float lastError;
	float dt;
	static float integral = 0.0;
	static float derivative = 0.0;

	float Kp = 10.0;
	float Ki = 50.0;
	float Kd = 0.5;
	static float pidOut = 0.0;

	struct vector sensor;

	const float K = 0.4;
	static float X = 0.0;
	float Xest;

	const float maxVel = 1.0;

	readSensors (&sensor.x, &sensor.y, &sensor.z);
	sensor.x -= sensorInit.x;
	sensor.y -= sensorInit.y;
	sensor.z -= sensorInit.z;

	dt = (time - lastTime);

	if (fabs (sensor.x - X) > fabs (maxVel * dt)) {
		puts ("bad data");
		goto endFunc;
	}

	Xest = X + ((velocity.x + pidOut) * dt);
	X = Xest + (K * (sensor.x - Xest));

	error = setPoint.x - X;
	integral += error * dt;
	derivative = (error - lastError) / dt;

	pidOut = (Ki * integral) + (Kp * error) + (Kd * derivative);

	printf ("%f, %f, %f, %f\n", sensor.x, X, Xest, pidOut);
	setVelocity (&xAxis, -velocity.x - pidOut);

	lastError = error;
	lastTime = time;

	endFunc:;

	struct timespec waitTime = {0, 5000000};
	nanosleep (&waitTime, NULL);
}

int compareFloats (const void *a, const void *b)
{
	const float *fa = (const float *)a;
	const float *fb = (const float *)b;

	return (*fa > *fb) - (*fa < *fb);
}

int main (int argc, char **argv)
{
	struct rampInfo ramp[2];
	struct lineInfo *line;
	struct arcInfo *arc;
	int lineNum;
	FILE *file;

	struct timeval progStart;

	float time = 0;
	float startTime;
	float diffTime;

        struct vector point = {0.0, 0.0, 0.0};
        struct vector velocity = {0.0, 0.0, 0.0};

	const int initPointNum = 100;
	float *initArrayX;
	float *initArrayY;
	float *initArrayZ;

	unsigned int i;
	unsigned int j;

	if (argc < 2) {
		fprintf (stderr, "You need to give a file name\n");
		return 1;
	} else if (argc > 2) {
		fprintf (stderr, "Too many arguments\n");
		return 1;
	}

	auto void cleanup (unsigned int success);

	tpruss_intc_initdata pruss_intc_initdata=PRUSS_INTC_INITDATA;

	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(PRU_NUM,"./sensor.bin");

	static void *sharedMem;
	prussdrv_map_prumem (PRUSS0_SHARED_DATARAM, &sharedMem);
	sharedMem_int = (unsigned int *)sharedMem;

	startupAxis (&xAxis, 30, 'x');
	startupAxis (&yAxis, 60, 'y');
	startupAxis (&zAxis, 31, 'z');

	waitEvent ();

	if (!(file = fopen (argv[1], "r"))) {
		perror ("Faild to open file");
		exit (1);
	}

	fread (&lineNum, sizeof(int), 1, file);

	fread (ramp, sizeof(struct rampInfo), 2, file);

	arc = malloc ((lineNum - 2) * sizeof(struct arcInfo));
	line = malloc ((lineNum - 1) * sizeof(struct lineInfo));

	fread (arc, sizeof(struct arcInfo), lineNum - 2, file);
	fread (line, sizeof(struct lineInfo), lineNum - 1, file);

	fclose (file);

	/*
	 * Finding the sensors zero point
	 */
	initArrayX = malloc (initPointNum * sizeof(*initArrayX));
	initArrayY = malloc (initPointNum * sizeof(*initArrayY));
	initArrayZ = malloc (initPointNum * sizeof(*initArrayZ));

	for (i = 0; i < initPointNum; i++) {
		struct timespec waitTime = {0, 5000000};
		readSensors (&initArrayX[i], &initArrayY[i], &initArrayZ[i]);
		nanosleep (&waitTime, NULL);
	}

	qsort (initArrayX, initPointNum, sizeof(*initArrayX), compareFloats);
	sensorInit.x = initArrayX[50];

	qsort (initArrayY, initPointNum, sizeof(*initArrayY), compareFloats);
	sensorInit.y = initArrayY[50];

	qsort (initArrayZ, initPointNum, sizeof(*initArrayZ), compareFloats);
	sensorInit.z = initArrayZ[50];

	free (initArrayX);
	free (initArrayY);
	free (initArrayZ);

	sleep (1);

	gettimeofday (&progStart, NULL);

	time = gettimefromfunction (progStart) * 1e-3;

	//Ramp up
	for (startTime = time, diffTime = 0.0; diffTime < ramp[0].time; time = gettimefromfunction (progStart) * 1e-3) {
		diffTime = time - startTime;
		point = multVec (ramp[0].accel, 0.5 * pow (diffTime, 2.0));
		velocity = multVec (ramp[0].accel, diffTime);
		printf ("%f, ", point.x);
		PID (point, velocity, time);
        }

        for (i = 0; ; i++) {
		//Line
		for (startTime = time, diffTime = 0.0; diffTime < line[i].time; time = gettimefromfunction (progStart) * 1e-3) {
			diffTime = time - startTime;
			point = addVec (line[i].start, multVec (line[i].velocity, diffTime));
			velocity = line[i].velocity;
			printf ("%f, ", point.x);
			PID (point, velocity, time);
		}

                if (i == lineNum - 2)
                        break;

		//Arc
		for (startTime = time, diffTime = 0.0; diffTime < arc[i].time; time = gettimefromfunction (progStart) * 1e-3) {
			float curAngle;
			float velMag;
			struct vector tangent;

			diffTime = time - startTime;

			curAngle = ((0.5 * arc[i].accel * pow (diffTime, 2.0)) + (magnitude (line[i].velocity) * diffTime)) / arc[i].radius;
                        point = getArcPos (arc[i], curAngle);

			printf ("%f, ", point.x);

			tangent = getArcPos (arc[i], curAngle + (M_PI / 2));
			tangent =  subVec (tangent, arc[i].center);
			normalize (&tangent);

			velMag = ((magnitude (line[i + 1].velocity) - magnitude (line[i].velocity)) * (diffTime / arc[i].time)) + magnitude (line[i].velocity);
			velocity = multVec (tangent, velMag);

			PID (point, velocity, time);
		}
        }

	//Ramp down
	for (startTime = time, diffTime = 0.0; diffTime < ramp[1].time; time = gettimefromfunction (progStart) * 1e-3) {
		diffTime = time - startTime;
		point = addVec (line[lineNum - 2].end, addVec (multVec (ramp[1].accel, -0.5 * pow (diffTime, 2.0)), multVec (line[lineNum - 2].velocity, diffTime)));
		velocity = addVec (multVec (ramp[1].accel, diffTime), line[lineNum - 2].velocity);
		printf ("%f, ", point.x);
		PID (point, velocity, time);
        }

	free (arc);
	free (line);

	exitFunction (0);

	void cleanup (unsigned int success)
	{
		if (success) {
			fprintf (stderr, "Bad exit\n");
		}

		shutdownAxis (&xAxis);
		shutdownAxis (&yAxis);
		shutdownAxis (&zAxis);

		prussdrv_pru_disable (PRU_NUM);
		prussdrv_exit ();
	}
}
