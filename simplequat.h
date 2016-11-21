/*
 * The MIT License
 *
 * Copyright 2016 Mihalis Tsiakkas.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef _QUATLIB_H_
#define _QUATLIB_H_

#include <cmath>
#include <iostream>
#include <exception>
#include <sstream>
#include <string.h>
#include <type_traits>

#define NUMERICAL_TOLERANCE 1E-6

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
    float operator[](int index) const;

    bool operator==(const Quaternion& q) const;
    bool operator!=(const Quaternion& q) const;
    bool operator>(const Quaternion& q) const;
    bool operator>=(const Quaternion& q) const;
    bool operator<(const Quaternion& q) const;
    bool operator<=(const Quaternion& q) const;
    
    int relaxedCompare(const Quaternion& q, float tolerance = NUMERICAL_TOLERANCE) const;
    
    Quaternion conjugate(void) const;
    Quaternion inverse(void) const;
    Quaternion& normalize(void);

    float* rotate3vector(const float* f);
    float* toEulerAngles(void);

    float getScalar(void) const { return *scalar_; }
    const float* getVector(void) const { return vector_; }
    const float* getData(void) const { return data_; }

    float norm(void) const;

    bool isNormalized(void) const { return fabs(norm() - 1.0f) < NUMERICAL_TOLERANCE; }
    bool isPure(void) const { return fabs(getScalar()) < NUMERICAL_TOLERANCE; }

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
