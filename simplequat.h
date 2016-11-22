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
#include <limits>

#define NUMERICAL_TOLERANCE 1E-8

class Quaternion {
public:

    enum ConstructorOptions {
        QuaternionData, EulerAngleData
    };

    Quaternion();
    Quaternion(const Quaternion& q);
    Quaternion(const double scalar, double vector1, double vector2, double vector3);

    Quaternion(const double scalar, const float * vector);
    Quaternion(const double scalar, const double * vector);
    Quaternion(const float * data, ConstructorOptions opt = QuaternionData);
    Quaternion(const double * data, ConstructorOptions opt = QuaternionData);
    
    ~Quaternion();

    Quaternion& operator=(const Quaternion& q);

    Quaternion operator+(const Quaternion& q) const;
    Quaternion operator-(const Quaternion& q) const;
    Quaternion operator-(void);
    Quaternion operator*(const Quaternion& q) const;
    Quaternion operator*(const double gam) const;
    Quaternion operator/(const double gam) const;

    Quaternion& operator+=(const Quaternion& q);
    Quaternion& operator-=(const Quaternion& q);
    Quaternion& operator*=(const Quaternion& q);
    Quaternion& operator*=(const double gam);
    Quaternion& operator/=(const double gam);

    double& operator[](int index);
    double operator[](int index) const;

    bool operator==(const Quaternion& q) const;
    bool operator!=(const Quaternion& q) const;
    bool operator>(const Quaternion& q) const;
    bool operator>=(const Quaternion& q) const;
    bool operator<(const Quaternion& q) const;
    bool operator<=(const Quaternion& q) const;
    
    int relaxedCompare(const Quaternion& q, double tolerance = TOLERANCE_) const;
    
    Quaternion conjugate(void) const;
    Quaternion inverse(void) const;
    Quaternion& normalize(void);

    double* rotateVector(const double* v);
    double* toEulerAngles(void);

    double getScalar(void) const { return *scalar_; }
    const double* getVector(void) const { return vector_; }
    const double* getData(void) const { return data_; }

    double norm(void) const;

    bool isNormalized(void) const { return fabs(norm() - 1.0f) < TOLERANCE_; }
    bool isPure(void) const { return fabs(getScalar()) < TOLERANCE_; }

    void setScalar(double scalar) { *(this->scalar_) = scalar; }
    void setVector(const double* vector);
    static void setNumericalTolerance(double tolerance) { TOLERANCE_ = fabs(tolerance); }

    std::string toString(void);
    void print(void);
    
    static Quaternion eulerToQuaternion(const double* euler);
    static Quaternion eulerToQuaternion(double roll, double pitch, double yaw);
    static double* quaternionToEuler(const Quaternion& q);
    
    static double* rotateVector(const double*  v, const Quaternion& q);
    
    static const Quaternion ZERO;
    static const Quaternion UNIT;

private:
    double* data_;
    double* scalar_; // Points to first element of data_
    double* vector_; // Points to second element of data_
    
    static double TOLERANCE_;
    
};

// EXCEPTION CLASSES

class quat_exc_division_by_zero : public std::exception {
public:

    virtual const char* what() const throw () override {
        return "QUATERNION::DIVISION BY ZERO";
    }
};

class quat_exc_not_normalized : public std::exception {
public:

    virtual const char* what() const throw () override {
        std::string err = "QUATERNION::NOT NORMALIZED ERR";
        return err.c_str();
    }
private:
    double norm;
};

class quat_exc_invalid_constructor_opts : public std::exception {
public:

    virtual const char* what() const throw () override {
        std::string err = "QUATERNION::INVALID CONSTRUCTOR OPTIONS";
        return err.c_str();
    }
};

class quat_exc_generic : public std::exception {
public:

    explicit quat_exc_generic(std::string err) : err(err) {
    }

    virtual const char* what() const throw () override {
        return err.c_str();
    }
private:
    std::string err;
};

#endif /* _QUATLIB_H_ */
