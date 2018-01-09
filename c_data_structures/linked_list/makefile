default: prog

linkedList.o: linkedList.c linkedList.h
	gcc -Wall -g -std=c99 -c linkedList.c	

linkedListMain.o: linkedListMain.c linkedList.h
	gcc -Wall -g -std=c99 -c linkedListMain.c

testLinkedList.o: linkedList.h testLinkedList.c
	gcc -Wall -g -std=c99 -c testLinkedList.c

prog: linkedList.o linkedListMain.o
	gcc -Wall -g -std=c99 -o prog linkedList.o linkedListMain.o
		
testLL: linkedList.o testLinkedList.o
	gcc -Wall -g -std=c99 -o testLL linkedList.o testLinkedList.o

clean:
	rm linkedList.o
	rm linkedListMain.o
	rm testLinkedList.o
