#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <setjmp.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_NUM 0
#define OFFSET_SHAREDRAM 2048

/*
 * Test to see if device tree is up
 * Motors are slow to respond
 * Jumps in position between lines and arcs
 * Only run when a new sensor reading comes in
 * Replace pwm with pru program
 * Error checking
 * Add y and z axies
 * Improve vector functions
 * Limit lines to 80 chars
 * Clean up this abomination
 */

struct axisInfo {
	FILE *run;
	FILE *duty;
	FILE *period;
	FILE *direction;
};

struct vector {
	float x;
	float y;
	float z;
};

struct arcInfo {
	struct vector toSurf;
	struct vector center;
	struct vector cross;
	float radius;
	float angle;
	float accel;
	float time;
};

struct lineInfo {
        struct vector velocity;
	struct vector start;
	struct vector end;
        float time;
};

struct rampInfo {
        struct vector accel;
        float time;
};

static jmp_buf buf;

static unsigned int *sharedMem_int;

struct axisInfo xAxis;
struct axisInfo yAxis;
struct axisInfo zAxis;

struct vector sensorInit = {0.0, 0.0, 0.0};

void signalCatcher (int null)
{
	longjmp (buf, 1);
}

int compareFloats (const void *a, const void *b)
{
	const float *fa = (const float *)a;
	const float *fb = (const float *)b;

	return (*fa > *fb) - (*fa < *fb);
}

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

void readPathFile (char *name, int *lineNum, struct rampInfo *ramp, struct arcInfo **arc, struct lineInfo **line)
{
	FILE *file;

	if (!(file = fopen (name, "r"))) {
		perror ("Failed to open file");
		exit (1);
	}

	fread (lineNum, sizeof(int), 1, file);

	fread (ramp, sizeof(struct rampInfo), 2, file);

	*arc = malloc ((*lineNum - 2) * sizeof(struct arcInfo));
	*line = malloc ((*lineNum - 1) * sizeof(struct lineInfo));

	fread (*arc, sizeof(struct arcInfo), *lineNum - 2, file);
	fread (*line, sizeof(struct lineInfo), *lineNum - 1, file);

	fclose (file);
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

void shutdownAxis (struct axisInfo *axis)
{
	fprintf (axis->run, "0");
	fclose (axis->run);

	fprintf (axis->duty, "0");
	fclose (axis->duty);

	fclose (axis->period);

	fclose (axis->direction);
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

void initSensors (void)
{
	unsigned int initPointNum = 100;
	float *initArrayX;
	float *initArrayY;
	float *initArrayZ;
	unsigned int i;

	initArrayX = malloc (initPointNum * sizeof(*initArrayX));
	initArrayY = malloc (initPointNum * sizeof(*initArrayY));
	initArrayZ = malloc (initPointNum * sizeof(*initArrayZ));

	for (i = initPointNum; i; i--) {
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
}

void setVelocity (struct axisInfo *axis, float *lastDir, float velocity)
{
	unsigned int period;
	unsigned int duty;

	if (velocity < 0) {
		velocity = fabs (velocity);
		if (*lastDir != 0) {
			*lastDir = 0;
			fprintf (axis->direction, "0");
			fflush (axis->direction);
		}
	} else {
		if (*lastDir != 1) {
			*lastDir = 1;
			fprintf (axis->direction, "1");
			fflush (axis->direction);
		}
	}

	period = (int)(500000.0 / velocity);
	duty = (int)(250000.0 / velocity);

	fprintf (axis->duty, "0");
	fflush (axis->duty);
	fprintf (axis->period, "%d", period);
	fflush (axis->period);
	fprintf (axis->duty, "%d", duty);
	fflush (axis->duty);
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

	struct timespec waitTime = {0, 5000000};

	static struct vector lastDir = {1.0, 1.0, 1.0};

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
	setVelocity (&xAxis, &lastDir.x, -velocity.x - pidOut);

	lastError = error;
	lastTime = time;

endFunc:

	nanosleep (&waitTime, NULL);
}

int main (int argc, char **argv)
{
	struct rampInfo ramp[2];
	struct lineInfo *line;
	struct arcInfo *arc;
	int lineNum;

	float time = 0;
	float startTime;
	float diffTime;

        struct vector point = {0.0, 0.0, 0.0};
        struct vector velocity = {0.0, 0.0, 0.0};

	struct timeval progStart;

	unsigned int i;

	if (setjmp (buf))
		goto shutdown;

	signal (SIGINT, signalCatcher);

	if (
		((argc < 2) ? fprintf (stderr, "You need to give a file name\n") : 0) ||
		((argc > 2) ? fprintf (stderr, "Too many arguments\n") : 0)
	)
		return 1;

	readPathFile (argv[1], &lineNum, ramp, &arc, &line);

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

	initSensors ();

	sleep (1);

	gettimeofday (&progStart, NULL);

	//Wouldn't this just be 0.0?
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

shutdown:

	free (arc);
	free (line);

	shutdownAxis (&xAxis);
	shutdownAxis (&yAxis);
	shutdownAxis (&zAxis);

	prussdrv_pru_disable (PRU_NUM);
	prussdrv_exit ();

	return 0;
}
