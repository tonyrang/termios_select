all:
	gcc -o serialport.a serialport.c
clean:
	rm -rf *.a