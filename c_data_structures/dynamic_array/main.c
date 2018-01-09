#include <stdio.h>
/*
 Author: Daniel Eisenbach and Jacob Fenger
 File: dynamicArray.c
 To do list program.
 */

#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "toDoList.h"


int main (int argc, const char * argv[])
{
	char cmd = ' ';
	char filename[100], description[250];
	DynArr* mainList = createDynArr(10);
	TYPE newEntry;
	TYPE min;
	int priority, ch;
	
	printf("\n\n** TO-DO LIST APPLICATION **\n");
	do{
		printf("\nPress:\n"
            "'l' to load to-do list from a file\n"
            "'s' to save to-do list to a file\n"
			"'a' to add a new task\n"
            "'g' to get the first task\n"
            "'r' to remove the first task\n"
            "'p' to print the list\n"
            "'e' to exit the program\n"
        );
		/* get input command (from the keyboard) */
		cmd = getchar();  
		/* clear the trailing newline character */
		while (getchar() != '\n');
		
		switch(cmd){
			case 'l':
				printf("\nPlease enter name of file to open: ");
				scanf("%s", filename);
				FILE *ifp;
				ifp = fopen(filename,"r");
				if (ifp == NULL) {
					fprintf(stderr, "\nCan't open input file!\n\n");
					break;
				}
				printf("\nYou entered %s\n", filename);
				loadList(mainList, ifp);
				fclose(ifp);
			break;
			
			case 's':
				printf("\nPlease enter a filename for the created file: ");
				scanf("%s", filename);
				FILE *ofp;
				ofp = fopen(filename,"w");
				if (ofp == NULL) {
					fprintf(stderr, "\nCan't make output file!\n\n");
					break;
				}
				printf("\nYou entered %s\n", filename);
				sortHeap(mainList, compare);
				saveList(mainList, ofp);
				fclose(ofp);
			break;
			
			case 'a':
				printf("\nPlease enter a task description for your new task: ");
				fgets(description, 250, stdin);
				printf("\nYou entered: %s\n", description);
				printf("\nPlease enter a priority for your new task: ");
				scanf("%d", &priority);
				printf("\nYou entered: %d\n", priority);
				
				newEntry = createTask(priority, description);
				addHeap(mainList, newEntry, compare);
			break;
			
			case 'g':
				min = getMinHeap(mainList);
				
				printf("\nThe first task in the list: \n");
				print_type(min);
			break;
			
			case 'r':
				if(isEmptyDynArr(mainList)){
					printf("\nError! To do list is empty!\n");
					break;
				}
				min = getMinHeap(mainList);
				
				printf("\nRemoving the first task. \n");
				removeMinHeap(mainList, compare);
			break;
			
			case 'p':
				printList(mainList);
			break;
		}
		if(cmd == 'e'){
			if(!isEmptyDynArr(mainList)){
				deleteDynArr(mainList); /* delete the list */
				printf("\nProgram memory successfully freed!\n\n");
			}
			exit(0);
		}
		else
			while((ch = getchar()) != '\n' && ch != EOF); /* clears input buffer */
    }while(cmd != 'e');
	
	return 0;
}
