//
//  cCircularBuffer.hpp
//
//  Created by Eric Trautmann on 3/31/16.
//  Copyright © 2016 Eric Trautmann. All rights reserved.
// based on code by Mahir Atmis from http://atmismahir.blogspot.com/2012/03/circular-buffer-c.html, accessed 3/31/2016

#ifndef _C_CIRCULAR_BUFFER_H
#define _C_CIRCULAR_BUFFER_H

#include <stdio.h>
#include <vector>


// This is a fixed-size, circular buffer of integer.
// The class is not thread-safe,
class cCircularBuffer
{
public:
    
	explicit cCircularBuffer();

    // Creates a buffer with 'slots' slots.
    explicit cCircularBuffer(int slots);
    // Destructor.
//   ~cCircularBuffer();
    // Writes 'value' to the next available slot. It may overwrite
    // values that were not yet read out of the buffer.
    void write(double value);
    // Returns the next value available for reading, in the order they
    // were written, and marks slot as read. If the buffer is empty returns -1.
    double read();
    
    void zero();
    
    double mean();
    
    long getLength();
    
    int getOrderedElements(std::vector<double> &out_vec);
    
    int filterSamples(double &sampleFiltered, std::vector<double> &coeff_vec);
    
    double getNBack(int n_back);
    
    void printBuffer(void);
    
    bool bufferFilled(void);
    
    
private:
    
    std::vector<double> data_vec;
    
    //array of integers
    double* data_;
    // the size of the buffer
    int  num_of_slots_;
    
    int num_of_filled_slots;
    
    //index to read the next integer from buffer
    int read_index_;
    
    //index to write a new integer to buffer
    int  write_index_;
    
    // Non-copyable, non-assignable.
//    cCircularBuffer(cCircularBuffer&);
//    cCircularBuffer& operator=(const cCircularBuffer&);
};


void printVector(std::vector<double> &vector);

#endif /* circularbuffer_hpp */




