#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <setjmp.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define OFFSET_SHAREDRAM 2048

/*
 * Test to see if device tree is up
 * Make sure that PRUWait is working
 * Motors are slow to respond
 * Jumps in position between lines and arcs
 * Only run when a new sensor reading comes in
 * Replace pwm with pru program
 * Error checking
 * Add y and z axies
 * Improve vector functions
 * Use vector functions more
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

struct vector divVec (struct vector vec, float scaler)
{
        return (struct vector){vec.x / scaler, vec.y / scaler, vec.z / scaler};
}

float magnitude (struct vector input)
{
        return fabs (sqrt (pow (input.x, 2) + pow (input.y, 2) + pow (input.z, 2)));
}

void normalize (struct vector *input)
{
	*input = divVec (*input, magnitude (*input));
}

struct vector getArcPos (struct arcInfo arc, float angle)
{
        float	cosAngle,
		sinAngle;

        cosAngle = cos (angle);
        sinAngle = sin (angle);

        return (struct vector){
                (cosAngle * arc.toSurf.x) + (sinAngle * arc.cross.x) + arc.center.x,
                (cosAngle * arc.toSurf.y) + (sinAngle * arc.cross.y) + arc.center.y,
                (cosAngle * arc.toSurf.z) + (sinAngle * arc.cross.z) + arc.center.z
        };
}

void PRUWait (void)
{
	prussdrv_pru_wait_event (PRU_EVTOUT_0);
	prussdrv_pru_wait_event (PRU_EVTOUT_0);
	prussdrv_pru_clear_event (PRU0_ARM_INTERRUPT);
}

void PRUInit (void)
{
	tpruss_intc_initdata pruss_intc_initdata=PRUSS_INTC_INITDATA;
	static void *sharedMem;

	prussdrv_init();

	if (prussdrv_open (PRU_EVTOUT_0) == -1) {
		puts ("Failed to initialize the PRU's memory mapping");
		exit (1);
	}

	if (prussdrv_pruintc_init (&pruss_intc_initdata) == -1) {
		puts ("Failed to initialize the PRU's interrupt controller");
		exit (1);
	}

	if (prussdrv_exec_program (0, "./sensor.bin") == -1) {
		puts ("Failed to execute PRU program");
		exit (1);
	}

	prussdrv_map_prumem (PRUSS0_SHARED_DATARAM, &sharedMem);
	sharedMem_int = (unsigned int *)sharedMem;
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

FILE *openLine (FILE *pipe, unsigned int length)
{
	char line[99];

	fgets (line, 99, pipe);
	line[length] = '\0';

	return fopen (line, "w");
}

void startupAxis (struct axisInfo *axis, unsigned int number, char letter)
{
	FILE *pipe;
	char bashCommand[22];

	sprintf (bashCommand, "./startupAxis.sh %d %c", number, letter);
	pipe = popen (bashCommand, "r");

	axis->run = openLine (pipe, 36);
	axis->duty = openLine (pipe, 37);
	axis->period = openLine (pipe, 39);
	axis->direction = openLine (pipe, 28);

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
		readSensors (&initArrayX[i], &initArrayY[i], &initArrayZ[i]);
		PRUWait ();
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

void kalman (struct vector *X, struct vector sensor, struct vector input, float dt)
{
	const float K = 0.4;
	struct vector Xest;

	Xest = addVec (*X, multVec (input, dt));
	*X = addVec (Xest, multVec (subVec (sensor, Xest), K));
}

struct vector PID (struct vector process, struct vector setPoint, float dt)
{
	const float Kp = 10.0;
	const float Ki = 50.0;
	const float Kd = 0.5;

	struct vector error;
	static struct vector lastError = {0.0, 0.0, 0.0};

	static struct vector integral = {0.0, 0.0, 0.0};
	struct vector derivative;

	error = subVec (setPoint, process);
	integral = addVec (integral, multVec (error, dt));
	derivative = divVec (subVec (error, lastError), dt);

	lastError = error;

	return (struct vector){
		(Kp * error.x) + (Ki * integral.x) + (Kd * derivative.x),
		(Kp * error.y) + (Ki * integral.y) + (Kd * derivative.y),
		(Kp * error.z) + (Ki * integral.z) + (Kd * derivative.z)
	};
}

int controls (struct vector setPoint, struct vector velocity, float time)
{
	static struct vector lastDir = {1.0, 1.0, 1.0};
	static struct vector pidOut = {0.0, 0.0, 0.0};
	static struct vector X = {0.0, 0.0, 0.0};
	static float lastTime = 0.0;
	const float maxVel = 1.0;
	struct vector sensor;
	float dt;

	readSensors (&sensor.x, &sensor.y, &sensor.z);
	sensor = subVec (sensor, sensorInit);

	dt = (time - lastTime);
	lastTime = time;

	if (fabs (sensor.x - X.x) > fabs (maxVel * dt)) {
		puts ("bad data");
		return 1;
	}

	kalman (&X, sensor, addVec (velocity, pidOut), dt);
	pidOut = PID (X, setPoint, dt);
	setVelocity (&xAxis, &lastDir.x, -velocity.x - pidOut.x);

	printf ("%f, %f, %f, %f\n", sensor.x, X.x, pidOut.x);

	return 0;
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

	startupAxis (&xAxis, 30, 'x');
	startupAxis (&yAxis, 60, 'y');
	startupAxis (&zAxis, 31, 'z');

	PRUInit ();
	PRUWait ();

	initSensors ();

	gettimeofday (&progStart, NULL);

	//Wouldn't this just be 0.0?
	time = gettimefromfunction (progStart) * 1e-3;

	//Ramp up
	for (startTime = time, diffTime = 0.0; diffTime < ramp[0].time; time = gettimefromfunction (progStart) * 1e-3) {
		diffTime = time - startTime;
		point = multVec (ramp[0].accel, 0.5 * pow (diffTime, 2.0));
		velocity = multVec (ramp[0].accel, diffTime);
		printf ("%f, ", point.x);
		controls (point, velocity, time);
		PRUWait ();
        }

        for (i = 0; ; i++) {
		//Line
		for (startTime = time, diffTime = 0.0; diffTime < line[i].time; time = gettimefromfunction (progStart) * 1e-3) {
			diffTime = time - startTime;
			point = addVec (line[i].start, multVec (line[i].velocity, diffTime));
			velocity = line[i].velocity;
			printf ("%f, ", point.x);
			controls (point, velocity, time);
			PRUWait ();
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

			controls (point, velocity, time);

			PRUWait ();
		}
        }

	//Ramp down
	for (startTime = time, diffTime = 0.0; diffTime < ramp[1].time; time = gettimefromfunction (progStart) * 1e-3) {
		diffTime = time - startTime;
		point = addVec (line[lineNum - 2].end, addVec (multVec (ramp[1].accel, -0.5 * pow (diffTime, 2.0)), multVec (line[lineNum - 2].velocity, diffTime)));
		velocity = addVec (multVec (ramp[1].accel, diffTime), line[lineNum - 2].velocity);
		printf ("%f, ", point.x);
		controls (point, velocity, time);
		PRUWait ();
        }

shutdown:

	free (arc);
	free (line);

	shutdownAxis (&xAxis);
	shutdownAxis (&yAxis);
	shutdownAxis (&zAxis);

	prussdrv_pru_disable (0);
	prussdrv_exit ();

	return 0;
}
