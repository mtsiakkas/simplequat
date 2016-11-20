/*
Copyright (c) 2016, Mihalis Tsiakkas
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _QUATLIB_H_
#define _QUATLIB_H_

#define NUM_TOLERANCE 10E-8

#include <math.h>

#include <iostream>
#include <exception>
#include <sstream>
#include <string>
#include <typeinfo>

class Quaternion {
public:

    enum ConstructorOptions {
        QuaternionData, EulerAngleData
    };

    Quaternion();
    Quaternion(const float scalar, const float * vector);
    Quaternion(const Quaternion& q);
    Quaternion(const float scalar, float vector1, float vector2, float vector3);
    Quaternion(const float* data, ConstructorOptions opt = QuaternionData);

    ~Quaternion();

    Quaternion& operator=(const Quaternion& q);

    Quaternion operator+(const Quaternion& q) const;
    Quaternion operator-(const Quaternion& q) const;
    Quaternion operator-(void);
    Quaternion operator*(const Quaternion& q) const;
    Quaternion operator*(const float gam) const;
    Quaternion operator/(const float gam) const;

    Quaternion& operator+=(const Quaternion& q);
    Quaternion& operator-=(const Quaternion& q);
    Quaternion& operator*=(const Quaternion& q);
    Quaternion& operator*=(const float gam);
    Quaternion& operator/=(const float gam);

    float& operator[](int index);

    bool operator==(const Quaternion& q) const;
    bool operator!=(const Quaternion& q) const;
    bool operator>(const Quaternion& q) const;
    bool operator>=(const Quaternion& q) const;
    bool operator<(const Quaternion& q) const;
    bool operator<=(const Quaternion& q) const;
    
    int relaxedCompare(const Quaternion& q, float tolerance = NUM_TOLERANCE) const;
    
    Quaternion conjugate(void) const;
    Quaternion inverse(void) const;
    Quaternion& normalize(void);

    float* rotate3vector(const float* f);
    float* toEulerAngles(void);

    float getScalar(void) const { return *scalar_; }
    const float* getVector(void) const { return vector_; }
    const float* getData(void) const { return data_; }

    float norm(void) const;

    bool isNormalized(void) const { return fabs(norm() - 1) < NUM_TOLERANCE; }
    bool isPure(void) const { return fabs(getScalar()) < NUM_TOLERANCE; }

    void setScalar(float scalar) { *(this->scalar_) = scalar; }
    void setVector(const float * vector);

    std::string toString(void);
    void print(void);

    static const Quaternion ZERO;
    static const Quaternion UNIT;

private:
    float* data_; // Used to keep scalar_ and vector_ consecutive in memory
    float* scalar_;
    float* vector_;
};

// EXCEPTION CLASSES

class quat_exc_division_by_zero : public std::exception {
public:

    virtual const char* what() const throw () {
        return "QUATERNION::DIVISION BY ZERO";
    }
};

class quat_exc_not_normalized : public std::exception {
public:

    explicit quat_exc_not_normalized(float norm) : norm(norm) {
    }

    virtual const char* what() const throw () {
        std::string err = "QUATERNION::NOT NORMALIZED ERR=" + \
        std::to_string(fabs(norm - 1));
        return err.c_str();
    }
private:
    float norm;
};

class quat_exc_invalid_constructor_opts : public std::exception {
public:

    virtual const char* what() const throw () {
        std::string err = "QUATERNION::INVALID CONSTRUCTOR OPTIONS";
        return err.c_str();
    }
};

class quat_exc_generic : public std::exception {
public:

    explicit quat_exc_generic(std::string err) : err(err) {
    }

    virtual const char* what() const throw () {
        return err.c_str();
    }
private:
    std::string err;
};

#endif /* _QUATLIB_H_ */
