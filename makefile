All:
	gcc -c control.c -o control.o
	gcc control.o -L/user/lib -lm -lprussdrv -lpthread -o control
	pasm -b sensor.asm | grep Error
	rm *~ && rm control.o
