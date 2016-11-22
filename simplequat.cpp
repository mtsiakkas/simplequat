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


#include "simplequat.h"

const Quaternion Quaternion::ZERO(0.0f, 0.0f, 0.0f, 0.0f);
const Quaternion Quaternion::UNIT(1.0f, 0.0f, 0.0f, 0.0f);
double Quaternion::TOLERANCE_ = 1E-8;

Quaternion::Quaternion() {
    *this = UNIT;
}

Quaternion::Quaternion(const Quaternion& q) {
    data_ = new double[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    memcpy(data_, q.data_, 4 * sizeof (double));
}

Quaternion::Quaternion(double scalar, double vector1, double vector2, double vector3) {
    data_ = new double[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    *scalar_ = scalar;
    vector_[0] = vector1;
    vector_[1] = vector2;
    vector_[2] = vector3;
}

Quaternion::Quaternion(const double scalar, const float * vector) {
    data_ = new double[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    *scalar_ = scalar;

    for (int i = 0; i < 3; i++) {
        vector_[i] = static_cast<double> (vector[i]);
    }
}

Quaternion::Quaternion(const double scalar, const double * vector) {
    data_ = new double[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    *scalar_ = scalar;

    memcpy(vector_, vector, 3 * sizeof (double));
}

Quaternion::Quaternion(const float * data, ConstructorOptions opt) {
    data_ = new double[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    switch (opt) {
        case QuaternionData:
            for (int i = 0; i < 4; i++) {
                data_[i] = static_cast<double> (data[i]);
            }
            break;
        case EulerAngleData:
        {
            double roll = data[1];
            double pitch = data[2];
            double yaw = data[3];

            memcpy(data_, eulerToQuaternion(roll, pitch, yaw).data_, 4 * sizeof (double));

            break;
        }
        default:
            throw quat_exc_invalid_constructor_opts();
            break;
    }
}

Quaternion::Quaternion(const double * data, ConstructorOptions opt) {
    data_ = new double[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    switch (opt) {
        case QuaternionData:
            memcpy(data_, data, 4 * sizeof (double));
            break;
        case EulerAngleData:
        {
            memcpy(data_, eulerToQuaternion(data).data_, 4 * sizeof (double));
            break;
        }
        default:
            throw quat_exc_invalid_constructor_opts();
            break;
    }
}

Quaternion::~Quaternion() {
    delete [] data_;

    data_ = nullptr;
    vector_ = nullptr;
    scalar_ = nullptr;
}

Quaternion& Quaternion::operator=(const Quaternion& q) {
    data_ = new double[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    memcpy(data_, q.data_, 4 * sizeof (double));

    return *this;
}

Quaternion Quaternion::operator+(const Quaternion& q) const {
    double newdata[4];

    for (int i = 0; i < 4; i++) {
        newdata[i] = data_[i] + q.data_[i];
    }

    return Quaternion(newdata);
}

Quaternion Quaternion::operator-(const Quaternion& q) const {
    double newdata[4];

    for (int i = 0; i < 4; i++) {
        newdata[i] = data_[i] - q.data_[i];
    }

    return Quaternion(newdata);
}

// Overload -- Multiplication by scalar

Quaternion Quaternion::operator*(const double gam) const {
    double newdata[4];

    for (int i = 0; i < 4; i++) {
        newdata[i] = data_[i] * gam;
    }

    return Quaternion(newdata);
}

void Quaternion::rotateVector(const double* vector_in, double* const vector_out) {
    rotateVector(vector_in, vector_out, *this);
}

// Overload -- Division by scalar

Quaternion Quaternion::operator/(const double gam) const {
    if (gam == 0) {
        throw quat_exc_division_by_zero();
    } else {
        double qdata[4];

        for (int i = 0; i < 4; i++) {
            qdata[i] = data_[i] / gam;
        }

        return Quaternion(qdata);
    }
}

Quaternion& Quaternion::operator+=(const Quaternion& q) {
    for (int i = 0; i < 4; i++) {
        data_[i] += q.data_[i];
    }

    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& q) {
    for (int i = 0; i < 4; i++) {
        data_[i] -= q.data_[i];
    }

    return *this;
}

Quaternion& Quaternion::operator*=(const double gam) {
    for (int i = 0; i < 4; i++) {
        data_[i] *= gam;
    }

    return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& q) {
    *this = *this * q;
    return *this;
}

Quaternion& Quaternion::operator/=(const double gam) {
    if (gam == 0) {
        throw quat_exc_division_by_zero();
    } else {
        for (int i = 0; i < 4; i++) {
            data_[i] /= gam;
        }
        return *this;
    }
}

// Unary negation

Quaternion Quaternion::operator-(void) {
    double qdata[4];

    for (int i = 0; i < 4; i++) {
        qdata[i] = -data_[i];
    }

    return Quaternion(qdata);
}

double& Quaternion::operator[](int index) {
    if (index > -1 && index < 4) {
        return data_[index];
    } else {
        throw quat_exc_generic("QUATERNION::INDEX OUT OF RANGE");
    }
}

double Quaternion::operator[](int index) const {
    if (index > -1 && index < 4) {
        return data_[index];
    } else {
        throw quat_exc_generic("QUATERNION::INDEX OUT OF RANGE");
    }
}

Quaternion Quaternion::conjugate(void) const {
    double vector[3];

    for (int i = 0; i < 3; i++) {
        vector[i] = -vector_[i];
    }

    return Quaternion(*scalar_, vector);
}

Quaternion Quaternion::inverse(void) const {
    return conjugate() / norm();
}

// Overload -- Quaternion product

Quaternion Quaternion::operator*(const Quaternion& q) const {
    const double* q_vector = q.vector_;
    double q_scalar = q[0];

    double scalar = *scalar_ * q_scalar;

    for (int i = 0; i < 3; i++) {
        scalar -= q_vector[i] * vector_[i];
    }

    double vector[3];

    vector[0] = *scalar_ * q_vector[0] + vector_[0] * q_scalar \
            + vector_[1] * q_vector[2] - vector_[2] * q_vector[1];
    vector[1] = *scalar_ * q_vector[1] + vector_[1] * q_scalar \
            - vector_[0] * q_vector[2] + vector_[2] * q_vector[0];
    vector[2] = *scalar_ * q_vector[2] + vector_[2] * q_scalar \
            + vector_[0] * q_vector[1] - vector_[1] * q_vector[0];

    return Quaternion(scalar, vector);
}

Quaternion& Quaternion::normalize(void) {
    if (*this == Quaternion::ZERO) {
        throw quat_exc_division_by_zero();
    } else {
        *this /= norm();
        return *this;
    }
}

bool Quaternion::operator==(const Quaternion& q) const {
    return memcmp(data_, q.data_, 4 * sizeof (double)) == 0;
}

bool Quaternion::operator!=(const Quaternion& q) const {
    return !(*this == q);
}

bool Quaternion::operator>(const Quaternion& q) const {
    return norm() > q.norm();
}

bool Quaternion::operator>=(const Quaternion& q) const {
    double norm_ = norm();
    double q_norm = q.norm();

    return norm_ > q_norm || norm_ == q_norm;
}

bool Quaternion::operator<(const Quaternion& q) const {
    return norm() < q.norm();
}

bool Quaternion::operator<=(const Quaternion& q) const {
    double norm_ = norm();
    double q_norm = q.norm();

    return norm_ < q_norm || norm_ == q_norm;
}

// Relaxed comparison function
// Inequalities evaluated based on norm()

int Quaternion::relaxedCompare(const Quaternion& q, double tolerance) const {

    int eq_count = 0;

    for (int i = 0; i < 4; i++) {
        if (fabs(data_[i] - q[i]) > fabs(tolerance)) {
            break;
        }
        eq_count++;
    }
    if (eq_count == 4) {
        return 0;
    }
    return norm() > q.norm() ? 1 : -1;
}

double Quaternion::norm(void) const {
    double norm_ = 0;
    for (int i = 0; i < 4; i++) {
        norm_ += data_[i] * data_[i];
    }
    return sqrt(norm_);
}

void Quaternion::setVector(const double * vector) {
    memcpy(vector_, vector, 3 * sizeof (double));
}

std::string Quaternion::toString(void) {
    std::stringstream ss;

    ss << "[";
    for (int i = 0; i < 4; i++) {
        ss << " " << data_[i];
    }
    ss << " ]" << std::endl;

    return ss.str().c_str();
}

void Quaternion::print(void) {
    std::cout << toString();
}

void Quaternion::toEulerAngles(double* const vector_out) {
    quaternionToEuler(vector_out, *this);
}

Quaternion Quaternion::eulerToQuaternion(const double * data) {
    double roll = data[1];
    double pitch = data[2];
    double yaw = data[3];

    return eulerToQuaternion(roll, pitch, yaw);
}

Quaternion Quaternion::eulerToQuaternion(double roll, double pitch, double yaw) {
    Quaternion qRoll(static_cast<double> (cos(roll / 2.0f)), static_cast<double> (sin(roll / 2.0f)), 0.0f, 0.0f);
    Quaternion qPitch(static_cast<double> (cos(pitch / 2.0f)), 0.0f, static_cast<double> (sin(pitch / 2.0f)), 0.0f);
    Quaternion qYaw(static_cast<double> (cos(yaw / 2.0f)), 0.0f, 0.0f, static_cast<double> (sin(yaw / 2.0f)));

    return Quaternion(qRoll * qPitch * qYaw);
}

void Quaternion::quaternionToEuler(double* const vector_out, const Quaternion& q) {
    //TODO: implement
}

void Quaternion::rotateVector(const double* vec_in, double* const vec_out, const Quaternion& q) {
    if (q.isNormalized()) {
        Quaternion fo = q * Quaternion(0, vec_in) * q.conjugate();
        memcpy(vec_out, fo.getVector(), 3 * sizeof (double));
    } else {
        throw quat_exc_not_normalized();
    }
}