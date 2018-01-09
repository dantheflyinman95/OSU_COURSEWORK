/* CS261- Assignment 3 - testLinkedList.c */
/* Name: Daniel Eisenbach and Jacob Fenger
 * Date: 4/27/2015
 * Solution description: test program for linked list implementation functions
 */
 
#include "linkedList.h"
#include <stdio.h>
#include <stdlib.h>

struct DLink {
	TYPE value;
	struct DLink * next;
	struct DLink * prev;
};

struct linkedList{
	int size;
	struct DLink *firstLink;
	struct DLink *lastLink;
};

int main(int argc, char* argv[]) {
	printf("\n\nHello from test code!\n");
	
	struct linkedList *lst = createLinkedList(); /* creating the linked list */
	printf("\n\nLinked list created for Deque implementation!\n");
	
	printf("\nTesting if list is starting empty: \n");
	if(isEmptyList(lst) == 0) /* Testing if the list empty */
		printf("The linked list is currently empty!\n");

	printf("\nTesting addFrontList() for the deque implementation! :");
	addFrontList(lst, 4); /* Added a four to the front of the list */
	printList(lst); 
	addFrontList(lst, 5);
	printList(lst); 
	addFrontList(lst, 6);
	printList(lst); 

	printf("\n\nValue at front of the list: %d", frontList(lst));
	printf("\nValue at back of the list: %d", backList(lst));

	printf("\n\nTesting addBackList() for the deque implementation! :");
	addBackList(lst, 3);
	printList(lst);
	addBackList(lst, 2);
	printList(lst);
	addBackList(lst, 1);
	printList(lst);

	printf("\n\nValue at front of the list: %d", frontList(lst));
	printf("\nValue at back of the list: %d", backList(lst));

	printf("\n\nTesting removeFrontList() for the deque implementation! :");
	removeFrontList(lst);
	printList(lst);

	printf("\n\nTesting removeBackList() for the deque implementation! :");
	removeBackList(lst);
	printList(lst);

	if(isEmptyList(lst) == 1){
		printf("\n\nThe linked list is not empty! \n");
	}

	printf("\nValue at front of the list: %d", frontList(lst));
	printf("\nValue at back of the list: %d", backList(lst));

	deleteLinkedList(lst);
	printf("\n\nLinked list for Deque implementation deleted!\n");
	
	struct linkedList *stack = createLinkedList(); /* creating the linked list */
	printf("\n\nLinked list created for stack implementation!\n");
	
	printf("\n\nTesting addList() for the stack implementation! :");
	addList(stack, 10);
	printList(stack);

	printf("\n\nTesting containsList() for the stack implementation! :");
	if(containsList(stack, 10) == 1)
		printf("\nThe value is inside the list!");
	else
		printf("\nThe value has not been found. ");
	printList(stack);

	printf("\n\nTesting removeList() for the stack implementation! :");
	removeList(stack, 10);
	printList(stack);

	deleteLinkedList(stack);
	printf("\n\nThe linked list stack implementation has been deleted! \n\n"); 

	return 0;
}

