/*
 * RingBuffer.h
 *
 *  Created on: Jun 3, 2015
 *      Author: mfiore
 *
 * 	Contains a RingBuffer, which is a template class able to keep n instances (to allocate at the start)
 *  of a certain type. Users can access a sequence of the last n instances. New instances erase the oldests.
 */

#ifndef SOURCE_DIRECTORY__SITUATION_ASSESSTMENT_SIMPLE_AGENT_MONITOR_SRC_RINGBUFFER_H_
#define SOURCE_DIRECTORY__SITUATION_ASSESSTMENT_SIMPLE_AGENT_MONITOR_SRC_RINGBUFFER_H_

#include <geometry_msgs/Pose.h>
#include <vector>

using namespace std;
template <class T>

/** 
*Contains a RingBuffer, which is a template class able to keep n instances (to allocate at the start)
*  of a certain type. Users can access a sequence of the last n instances. New instances erase the oldests.
*/
class RingBuffer {
public:
	/**
	 * Constructor of a RingBuffer
	 */
	RingBuffer();
	virtual ~RingBuffer();

	/**
	 * Inserts an element in the RingBuffer
	 * @param element elemente to insert
	 */
	void insert(T element);

	/**
	 * Get the last n elements in the RingBuffer
	 */
	vector<T> getSequence(int n);  

	/**
	 * Allocates space for the RingBuffer
	 * @param size size of the RingBuffer
	 */
	void allocate(int size);

	/**
	 * Checks if the ringbuffer is empty
	 * @return true if it is empty
	 */
	bool isEmpty();



private:
	int size;
	
	vector<T> elements;
	int last;
};


template <class T> bool RingBuffer<T>::isEmpty() {
	return size==0;
}
template <class T>
RingBuffer<T>::RingBuffer() {
	// TODO Auto-generated constructor stub
	last=0;
	this->size=0;
}

template <class T>
RingBuffer<T>::~RingBuffer() {
	// TODO Auto-generated destructor stub
}
template <class T>
void RingBuffer<T>::allocate(int size) {
	for (int i=0; i<size;i++) {
		T p;
		elements.push_back(p);
	}
	this->size=size;
}

template <class T>
void RingBuffer<T>::insert(T newElement) {
	last++;
	if (last==size) {
		last=0;
	}
	elements[last]=newElement;

}

template <class T>
vector<T>  RingBuffer<T>::getSequence(int n){
	vector<T> result;

	int nElements=0;
	int i=last;
	while(nElements<n) {
		result.push_back(elements[i]);
		nElements++;

		i--;
		if (i<0) {
			i=n-1;
		}
	}
	return result;

}








#endif /* SOURCE_DIRECTORY__SITUATION_ASSESSTMENT_SIMPLE_AGENT_MONITOR_SRC_RINGBUFFER_H_ */
