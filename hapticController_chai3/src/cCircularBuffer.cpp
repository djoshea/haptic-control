//
//  cCircularBuffer.cpp
//
//  Created by Eric Trautmann on 3/31/16.
//  Copyright © 2016 Eric Trautmann. All rights reserved.
//
// based on code by Mahir Atmis from http://atmismahir.blogspot.com/2012/03/circular-buffer-c.html, accessed 3/31/2016

#include "cCircularBuffer.hpp"

cCircularBuffer::cCircularBuffer() {
	write_index_ = 0;
	read_index_ = -1;
	num_of_filled_slots = 0;
}

cCircularBuffer::cCircularBuffer(int slots) : data_vec(slots, 0) {
    if (slots <= 0) {
        num_of_slots_ = 10; /*pre-assigned value */
    } else {
        num_of_slots_ = slots;
    }
    
    write_index_ = 0;
    read_index_ = -1;
    num_of_filled_slots = 0;
    
}

void cCircularBuffer::zero() {
    for(int ii = 0; ii < num_of_slots_; ii++){
        data_vec[ii] = 0;
    }
}

//cCircularBuffer::~cCircularBuffer() {
//    delete[] data_;
//}

void cCircularBuffer::write(double value) {
    data_vec[write_index_] = value;
    if (read_index_ == -1) {
        //if buffer is empty, set the read index to the
        //current write index. because that will be the first
        //slot to be read later.
        read_index_ = write_index_;
    }
    write_index_ = (write_index_ + 1) % num_of_slots_;
    
    if (num_of_filled_slots < num_of_slots_) {
        num_of_filled_slots += 1;
    }
}


double cCircularBuffer::mean() {
    double sum = 0;
    for (int ii = 0; ii < num_of_slots_; ii++){
        sum += data_vec[ii];
    }
    return sum/(double)num_of_slots_;
}

long cCircularBuffer::getLength() {
    return data_vec.size();
}


int cCircularBuffer::getOrderedElements(std::vector<double> &out_vec) {
    if (out_vec.size() != data_vec.size()) {
        printf("vector size mismatch - input vector must be %lu elements\n", data_vec.size());
        return -1;
    }
//    printf("wi: %d, ", write_index_);
    for (int ii = 0; ii < data_vec.size(); ii++) {
        int nBackInd = (write_index_ - 1 - ii);
        
        // % operator giving results that don't agree with matlab's mod(), do this manually
        // I'm sure there's a better way (EMT)
        if (nBackInd < 0) {
            nBackInd += (int)data_vec.size();
        }
//        printf("nbi: %d, ", nBackInd);
        out_vec.at(ii) = data_vec[nBackInd];
    }
    return 0;  //#todo remove

}

// apply a provided filter to
int cCircularBuffer::filterSamples(double &sampleFiltered, std::vector<double> &coeff_vec) {
    
    if (coeff_vec.size() > data_vec.size()) {
        printf("error: vector of oefficients longer than circular buffer");
        return -1;
    }
    
    // multiply accumulate through samples starting from most recent
    long filter_length = coeff_vec.size();
    
    sampleFiltered = 0;
    for (int ii = 0; ii < filter_length; ii++) {
        double thisCoeff = coeff_vec.at(ii);
        
        int nBackInd = (write_index_ - 1 - ii);
        // % operator giving results that don't agree with matlab's mod(), do this manually
        // I'm sure there's a better way (EMT)
        if (nBackInd < 0) {
            nBackInd += (int)data_vec.size();
        }

        double thisSamp = data_vec[nBackInd];
        sampleFiltered += thisCoeff*thisSamp;
    }
    sampleFiltered = sampleFiltered/filter_length;
    
    return 0;
}

// for debugging
void cCircularBuffer::printBuffer(void) {
    printf(" buffer:  ");
    for (int ii = 0; ii < data_vec.size(); ii++) {
        printf("%.1f, ", data_vec[ii]);
    }
}

bool cCircularBuffer::bufferFilled() {
    if (num_of_filled_slots < num_of_slots_) {
        return false;
    }
    else {
        return true;
    }
}

// for debugging
void printVector(std::vector<double> &vector) {
    printf(" elements:  ");
    for (int ii = 0; ii < vector.size(); ii++) {
        printf("%.1f, ", vector[ii]);
    }
    printf("\n");
}
