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

#define PRU_NUM 0
#define OFFSET_SHAREDRAM 2048

#define exitFunction(a) \
	cleanup (a); \
	return a;

/*
 * Get rid of exitFunction macro
 * Repace file functions with bash
 * Add pid function
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
        double	radius,
		angle,
		accel,
		time;
};

struct lineInfo {
        struct vector	velocity,
        		start,
        		end;
        double time;
};

struct rampInfo {
        struct vector accel;
        double time;
};

static unsigned int *sharedMem_int;

struct vector addVec (struct vector in1, struct vector in2)
{
        return (struct vector){in1.x + in2.x, in1.y + in2.y, in1.z + in2.z};
}

struct vector multVec (struct vector vec, double scaler)
{
        return (struct vector){vec.x * scaler, vec.y * scaler, vec.z * scaler};
}

double magnitude (struct vector input)
{
        return fabs (sqrt (pow (input.x, 2) + pow (input.y, 2) + pow (input.z, 2)));
}

struct vector getArcPos (struct arcInfo arc, double angle)
{
        double  cosAngle,
                sinAngle;

        cosAngle = cos (angle);
        sinAngle = sin (angle);

        return (struct vector){
                (cosAngle * arc.toSurf.x) + (sinAngle * arc.cross.x) + arc.center.x,
                (cosAngle * arc.toSurf.y) + (sinAngle * arc.cross.y) + arc.center.y,
                (cosAngle * arc.toSurf.z) + (sinAngle * arc.cross.z) + arc.center.z
        };
}

double gettimefromfunction (struct timeval startTime)
{
	struct timeval curTime;

	gettimeofday (&curTime, NULL);

	return ((double)(curTime.tv_sec - startTime.tv_sec) * 1e3) + ((double)(curTime.tv_usec - startTime.tv_usec) * 1e-3);
}

unsigned int getFd (FILE **file, char *path, char *name)
{
	auto void cleanup (unsigned int success);

	char *subPath;
	glob_t globPath;
	FILE *fd;

	if (!(subPath = malloc (2 + strlen (path) + strlen (name)))) {exitFunction (1);}
	if ((snprintf (subPath, 2 + strlen (path) + strlen (name), "%s/%s", path, name)) < 0) {exitFunction (1);}
	if (!(*file = fopen (subPath, "w"))) {exitFunction (1);}

	exitFunction (0);

	void cleanup (unsigned int success)
	{
		if (subPath)
			free (subPath);
	}
}

unsigned int startupAxis (struct axisInfo *axis, unsigned int number, char letter)
{
	auto void cleanup (unsigned int success);

	char *path;
	glob_t globPath;
	unsigned int numChar;
	char *gpioPath;

	axis->run = NULL;
	axis->duty = NULL;
	axis->period = NULL;
	axis->direction = NULL;

	if (!(path = malloc (34))){exitFunction(1);}
	if ((snprintf (path, 34, "/sys/devices/ocp.*/%c_axis_pwm.*", letter)) < 0) {exitFunction(1);}
	if (glob (path, 0, NULL, &globPath)) {exitFunction(1);}

	if (getFd (&axis->run, globPath.gl_pathv[0], "run")) {exitFunction(1);}
	if (getFd (&axis->duty, globPath.gl_pathv[0], "duty")) {exitFunction(1);}
	if (getFd (&axis->period, globPath.gl_pathv[0], "period")) {exitFunction(1);}

	if (number < 10)
		numChar = 1;
	else if (number < 100)
		numChar = 2;
	else if (number < 1000)
		numChar = 3;
	else {
		exitFunction (1);
	}

	if (!(gpioPath = malloc (27 + numChar))) {exitFunction(1);}
	if ((snprintf (gpioPath, 27 + numChar, "/sys/class/gpio/gpio%d/value", number)) < 0) {exitFunction(1);}
	if (!(axis->direction = fopen (gpioPath, "w"))) {exitFunction(1);}

	if ((fprintf (axis->duty, "0")) < 0) {exitFunction(1);}
	if (fflush (axis->duty)) {exitFunction(1);}

	if ((fprintf (axis->run, "1")) < 0) {exitFunction(1);}
	if (fflush (axis->run)) {exitFunction(1);}

	if ((fprintf (axis->direction, "1")) < 0) {exitFunction(1);}
	if (fflush (axis->direction)) {exitFunction(1);}

	exitFunction (0);

	void cleanup (unsigned int success)
	{
		if (success) {
			fprintf (stderr, "Failed to initalize the %c axis, have you applied the device tree overlay?\n", letter);
		}

		if (path)
			free (path);

		globfree (&globPath);
	}
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

/*
 * Velocity is measured in inches per giga-seconds
 */
unsigned int setVelocity (struct axisInfo *axis, int velocity)
{
	static unsigned int direction = 1;
	unsigned int speed;
	unsigned int period;
	unsigned int duty;

	if (velocity < 0) {
		speed = abs (velocity);
		if (direction != 0) {
			direction = 0;
			if ((fprintf (axis->direction, "0")) < 0) {return 1;}
			if (fflush (axis->direction)) {return 1;}
		}
	} else {
		speed = velocity;
		if (direction != 1) {
			direction = 1;
			if ((fprintf (axis->direction, "1")) < 0) {return 1;}
			if (fflush (axis->direction)) {return 1;}
		}
	}

	period = 500000000000000 / speed;
	duty = 250000000000000 / speed;

	if ((fprintf (axis->duty, "0")) < 0) {return 1;}
	if (fflush (axis->duty)) {return 1;}
	if ((fprintf (axis->period, "%d", period)) < 0) {return 1;}
	if (fflush (axis->period)) {return 1;}
	if ((fprintf (axis->duty, "%d", duty)) < 0) {return 1;}
	if (fflush (axis->duty)) {return 1;}

	return 0;
}

void PID (struct vector setPoint, double time)
{
	static double lastTime = 0.0;
	static double error = 0.0;
	double lastError;
	double dt;
	static double integral = 0.0;
	static double derivative = 0.0;

	double Kp = 1.0;
	double Ki = 0.0;
	double Kd = 0.0;
	double pidOut;

	struct vector sensor;

	readSensors (&sensor.x, &sensor.y, &sensor.z);

	error = setPoint.x - sensor.x;
	dt = time - lastTime;
	integral += error * dt;
	derivative = (error - lastError) / dt;

	pidOut = (Ki * integral) + (Kp * error) + (Kd * derivative);

	printf ("%lf, %lf\n", setPoint.x, sensor.x);
	//printf ("%lf\n", pidOut);
	//printf ("%d\n", pidOut * 100000000);
	//if (setVelocity (&xAxis, -100000000)) {exitFunction (1);}
	setVelocity (&xAxis, (int)(pidOut * -100000000));

	lastError = error;
	lastTime = time;
}

int main (int argc, char **argv)
{
	struct rampInfo ramp[2];
	struct lineInfo *line;
	struct arcInfo *arc;
	int lineNum;
	FILE *file;

	struct timeval progStart;

	double time = 0;
	double startTime;
	double diffTime;

        struct vector point;

	unsigned int i;

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

	if (startupAxis (&xAxis, 30, 'x')) {exitFunction (1);}
	if (startupAxis (&yAxis, 60, 'y')) {exitFunction (1);}
	if (startupAxis (&zAxis, 31, 'z')) {exitFunction (1);}

	waitEvent ();

	float xZero = 0;
	float yZero = 0;
	float zZero = 0;

	float sensorX, sensorY, sensorZ;

	file = fopen (argv[1], "r");

	fread (&lineNum, sizeof(int), 1, file);

	fread (ramp, sizeof(struct rampInfo), 2, file);

	arc = malloc ((lineNum - 2) * sizeof(struct arcInfo));
	line = malloc ((lineNum - 1) * sizeof(struct lineInfo));

	fread (arc, sizeof(struct arcInfo), lineNum - 2, file);
	fread (line, sizeof(struct lineInfo), lineNum - 1, file);

	fclose (file);

	gettimeofday (&progStart, NULL);

	while (1) {
		struct vector point = {1.0, 0.0, 0.0};
		time = gettimefromfunction (progStart);
		PID (point, time);
	}

	/*
	if (setVelocity (&xAxis, -100000000)) {exitFunction (1);}
	while (1) {
		readSensors (&sensorX, &sensorY, &sensorZ);
		printf ("%f, %f, %f\n", sensorX, sensorY, sensorZ);
	}
	*/

	/*
	//Ramp up
	for (startTime = time, diffTime = 0.0; diffTime < ramp[0].time; time = gettimefromfunction (progStart)) {
		diffTime = time - startTime;
		point = multVec (ramp[0].accel, 0.5 * pow (diffTime, 2.0));
        }

        for (i = 0; ; i++) {
		//Line
		for (startTime = time, diffTime = 0.0; diffTime < line[i].time; time = gettimefromfunction (progStart)) {
			diffTime = time - startTime;
			point = addVec (line[i].start, multVec (line[i].velocity, diffTime));
		}

                if (i == lineNum - 2)
                        break;

		//Arc
		for (startTime = time, diffTime = 0.0; diffTime < arc[i].time; time = gettimefromfunction (progStart)) {
			diffTime = time - startTime;
                        point = getArcPos (arc[i], ((0.5 * arc[i].accel * pow (diffTime, 2.0)) + (magnitude (line[i].velocity) * diffTime)) / arc[i].radius);
		}
        }

	//Ramp down
	for (startTime = time, diffTime = 0.0; diffTime < ramp[1].time; time = gettimefromfunction (progStart)) {
		diffTime = time - startTime;
		point = addVec (line[lineNum - 2].end, addVec (multVec (ramp[1].accel, -0.5 * pow (diffTime, 2.0)), multVec (line[lineNum - 2].velocity, diffTime)));
        }
	*/

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
