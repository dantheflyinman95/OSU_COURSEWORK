/* CS261- Assignment 3 - linkedList.c */
/* Name: Daniel Eisenbach and Jacob Fenger
 * Date: 4/27/2015
 * Solution description: linked list deque and stack implementation
 */

#include "linkedList.h"
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>


/* Double Link*/
struct DLink {
	TYPE value;
	struct DLink * next;
	struct DLink * prev;
};

/* Double Linked List with Head and Tail Sentinels  */

struct linkedList{
	int size;
	struct DLink *firstLink;
	struct DLink *lastLink;
};

/*
	initList
	param lst the linkedList
	pre: lst is not null
	post: lst size is 0
*/

void _initList (struct linkedList *lst) {
	if(lst == NULL)
		exit(0);
	lst->firstLink = malloc(sizeof(struct DLink));
	assert(lst->firstLink != 0);
  
	lst->lastLink = malloc(sizeof(struct DLink));
	assert(lst->lastLink != 0);
  
	lst->firstLink->next = lst->lastLink;
	lst->lastLink->prev = lst->firstLink;
	lst->size = 0;
}

/*
 createList
 param: none
 pre: none
 post: firstLink and lastLink reference sentinels
 */

struct linkedList *createLinkedList()
{
	struct linkedList *newList = malloc(sizeof(struct linkedList));
	_initList(newList);
	return(newList);
}

/*
	_addLinkBefore
	param: lst the linkedList
	param: l the  link to add before
	param: v the value to add
	pre: lst is not null
	pre: l is not null
	post: lst is not empty
*/

/* Creates and adds a new link before the provided link, l */

void _addLinkBefore(struct linkedList *lst, struct DLink *l, TYPE v)
{
	if(lst == NULL || l == NULL)
		exit(0);
	
	struct DLink *newLink = malloc(sizeof(struct DLink));
	newLink->value = v;
	newLink->next = l;
	newLink->prev = l->prev;
	newLink->prev->next = newLink;
	l->prev = newLink;
	
	lst->size++;
}

/*
	_removeLink
	param: lst the linkedList
	param: l the link to be removed
	pre: lst is not null
	pre: l is not null
	post: lst size is reduced by 1
*/
void _removeLink(struct linkedList *lst, struct DLink *l)
{
	if(lst == NULL || l == NULL)
		exit(0);
	
	if(l->next != NULL) /* insures node to be deleted is not last node */
		l->next->prev = l->prev;
	
	if(l->prev != NULL) /* insures node to be deleted is not first node */
		l->prev->next = l->next;
		
	lst->size--;
	
	free(l);
}

/*
	isEmptyList
	param: lst the linkedList
	pre: lst is not null
	post: none
*/
int isEmptyList(struct linkedList *lst) {
	if(lst == NULL)
		exit(0);
	
	if(lst->firstLink->next == lst->lastLink && lst->lastLink->prev == lst->firstLink)
		return(0); /* returns 0 if list is empty */
	
	return(1); /* returns 1 if list is not empty */
}

/* De-allocate all links of the list

	param: 	lst		pointer to the linked list
	pre:	none
	post:	All links (including the two sentinels) are de-allocated
*/
void freeLinkedList(struct linkedList *lst)
{
	while(!isEmptyList(lst)) 
		_removeLink(lst, lst->firstLink->next); /* remove the link right after the first sentinel */	
	
	/* remove the first and last sentinels */
	free(lst->firstLink);
	free(lst->lastLink);	
}

/* 	Deallocate all the links and the linked list itself. 

	param: 	v		pointer to the dynamic array
	pre:	v is not null
	post:	the memory used by v->data is freed
*/
void deleteLinkedList(struct linkedList *lst)
{
	if(lst == NULL)
		exit(0);
	
	freeLinkedList(lst);
	free(lst);
}


/* Function to print list
 Pre: lst is not null
 */
void printList(struct linkedList* lst)
{
	if(lst == NULL)
		exit(0);
	
	struct DLink *current;
	current = lst->firstLink->next;
	printf("\nLinked List Values: \n");
	while(current != lst->lastLink){
		printf(" %d ", current->value);
		current = current->next;
	}
	printf("\n");
}

/* ************************************************************************
	Deque Interface Functions
************************************************************************ */

/*
	addFrontList
	param: lst the linkedList
	param: e the element to be added
	pre: lst is not null
	post: lst is not empty, increased size by 1
*/
void addFrontList(struct linkedList *lst, TYPE e)
{
	if(lst == NULL)
		exit(0);
	
	_addLinkBefore(lst,lst->firstLink->next,e);
}

/*
	addBackList
	param: lst the linkedList
	param: e the element to be added
	pre: lst is not null
	post: lst is not empty, increased size by 1
*/
void addBackList(struct linkedList *lst, TYPE e) {
	if(lst == NULL)
		exit(0);
	
	_addLinkBefore(lst,lst->lastLink,e);
}

/*
	frontList
	param: lst the linkedList
	pre: lst is not null
	pre: lst is not empty
	post: none
*/
TYPE frontList (struct linkedList *lst) {
	if(lst == NULL || lst->size == 0)
		exit(0);
	
	return(lst->firstLink->next->value);
}

/*
	backList
	param: lst the linkedList
	pre: lst is not null
	pre: lst is not empty
	post: lst is not empty
*/
TYPE backList(struct linkedList *lst)
{
	if(lst == NULL || lst->size == 0)
		exit(0);
	
	return(lst->lastLink->prev->value);
}

/*
	removeFrontList
	param: lst the linkedList
	pre:lst is not null
	pre: lst is not empty
	post: size is reduced by 1
*/
void removeFrontList(struct linkedList *lst) {
	if(lst == NULL || lst->size == 0)
		exit(0);
	
	_removeLink(lst,lst->firstLink->next);
}

/*
	removeBackList
	param: lst the linkedList
	pre: lst is not null
	pre:lst is not empty
	post: size reduced by 1
*/
void removeBackList(struct linkedList *lst)
{	
	if(lst == NULL || lst->size == 0)
		exit(0);
	
	_removeLink(lst,lst->lastLink->prev);
}


/* ************************************************************************
	Stack Interface Functions
************************************************************************ */

/* 
	Add an item to the bag
	param: 	lst		pointer to the bag
	param: 	v		value to be added
	pre:	lst is not null
	post:	a link storing val is added to the bag
 */
void addList(struct linkedList *lst, TYPE v)
{
	if(lst == NULL)
		exit(0);
	
	_addLinkBefore(lst,lst->firstLink->next,v);
}

/*	Returns boolean (encoded as an int) demonstrating whether or not
	the specified value is in the collection
	true = 1
	false = 0

	param:	lst		pointer to the bag
	param:	e		the value to look for in the bag
	pre:	lst is not null
	pre:	lst is not empty
	post:	no changes to the bag
*/
int containsList (struct linkedList *lst, TYPE e) {
	if(lst == NULL || lst->size == 0)
		exit(0);
	
	struct DLink *current;
	current = lst->firstLink->next;
	while(current != lst->lastLink){
		if(current->value == e)
			return 1;
		else
			current = current->next;
	}
	return 0;
}

/*	Removes the first occurrence of the specified value from the collection
	if it occurs

	param:	lst		pointer to the bag
	param:	e		the value to be removed from the bag
	pre:	lst is not null
	pre:	lst is not empty
	post:	e has been removed
	post:	size of the bag is reduced by 1
*/
void removeList (struct linkedList *lst, TYPE e) {
	if(lst == NULL || lst->size == 0)
		exit(0);
	
	if(containsList(lst,e) == 0){
		printf("\n\nError! Value is not in the list! Exiting.\n");
		exit(0);
	}
	
	struct DLink *current;
	current = lst->firstLink->next;
	while(current != lst->lastLink){
		if(current->value == e)
			_removeLink(lst,current);
		else
			current = current->next;
	}
}
