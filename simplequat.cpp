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

Quaternion::Quaternion() {
    *this = UNIT;
}

Quaternion::Quaternion(const Quaternion& q) {
    data_ = new float[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    memcpy(data_, q.data_, 4 * sizeof (float));
}

Quaternion::Quaternion(float scalar, float vector1, float vector2, float vector3) {
    data_ = new float[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    *scalar_ = scalar;
    vector_[0] = vector1;
    vector_[1] = vector2;
    vector_[2] = vector3;
}

Quaternion::Quaternion(const float scalar, const float * vector) {
    data_ = new float[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    *scalar_ = scalar;
    memcpy(vector_, vector, 3 * sizeof (float));
}

Quaternion::Quaternion(const float* data, ConstructorOptions opt) {

    data_ = new float[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    switch (opt) {
        case QuaternionData:
            memcpy(data_, data, 4 * sizeof (float));
            break;
        case EulerAngleData:
        {

            float roll = data[1];
            float pitch = data[2];
            float yaw = data[3];

            Quaternion qRoll(static_cast<float> (cos(roll / 2.0f)), static_cast<float> (sin(roll / 2.0f)), 0.0f, 0.0f);
            Quaternion qPitch(static_cast<float> (cos(pitch / 2.0f)), 0.0f, static_cast<float> (sin(pitch / 2.0f)), 0.0f);
            Quaternion qYaw(static_cast<float> (cos(yaw / 2.0f)), 0.0f, 0.0f, static_cast<float> (sin(yaw / 2.0f)));
            Quaternion qTmp = qRoll * qPitch * qYaw;

            memcpy(data_, qTmp.data_, 4 * sizeof (float));

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
    data_ = new float[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    memcpy(data_, q.data_, 4 * sizeof (float));

    return *this;
}

Quaternion Quaternion::operator+(const Quaternion& q) const {
    float newdata[4];

    for (int i = 0; i < 4; i++) {
        newdata[i] = data_[i] + q.data_[i];
    }

    return Quaternion(newdata);
}

Quaternion Quaternion::operator-(const Quaternion& q) const {
    float newdata[4];

    for (int i = 0; i < 4; i++) {
        newdata[i] = data_[i] - q.data_[i];
    }

    return Quaternion(newdata);
}

// Overload -- Multiplication by scalar

Quaternion Quaternion::operator*(const float gam) const {
    float newdata[4];

    for (int i = 0; i < 4; i++) {
        newdata[i] = data_[i] * gam;
    }

    return Quaternion(newdata);
}

float* Quaternion::rotate3vector(const float* f) {
    if (isNormalized()) {
        float* out = new float[3];
        Quaternion fo = *this * Quaternion(0, f) * (this->conjugate());
        memcpy(out, fo.vector_, 3 * sizeof (float));

        return out;
    } else {
        throw quat_exc_not_normalized(norm());
    }
}

// Overload -- Division by scalar

Quaternion Quaternion::operator/(const float gam) const {
    if (gam == 0) {
        throw quat_exc_division_by_zero();
    } else {
        float qdata[4];

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

Quaternion& Quaternion::operator*=(const float gam) {
    for (int i = 0; i < 4; i++) {
        data_[i] *= gam;
    }

    return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& q) {
    *this = *this * q;
    return *this;
}

Quaternion& Quaternion::operator/=(const float gam) {
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
    float qdata[4];

    for (int i = 0; i < 4; i++) {
        qdata[i] = -data_[i];
    }

    return Quaternion(qdata);
}

float& Quaternion::operator[](int index) {
    if (index > -1 && index < 4) {
        return data_[index];
    } else {
        throw quat_exc_generic("QUATERNION::INDEX OUT OF RANGE");
    }
}

float Quaternion::operator[](int index) const {
    if (index > -1 && index < 4) {
        return data_[index];
    } else {
        throw quat_exc_generic("QUATERNION::INDEX OUT OF RANGE");
    }
}

Quaternion Quaternion::conjugate(void) const {
    float vector[3];

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
    const float* q_vector = q.vector_;
    float q_scalar = q[0];

    float scalar = *scalar_ * q_scalar;

    for (int i = 0; i < 3; i++) {
        scalar -= q_vector[i] * vector_[i];
    }

    float vector[3];

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
    return memcmp(data_, q.data_, 4 * sizeof (float)) == 0;
}

bool Quaternion::operator!=(const Quaternion& q) const {
    return !(*this == q);
}

bool Quaternion::operator>(const Quaternion& q) const {
    return norm() > q.norm();
}

bool Quaternion::operator>=(const Quaternion& q) const {
    float norm_ = norm();
    float q_norm = q.norm();

    return norm_ > q_norm || norm_ == q_norm;
}

bool Quaternion::operator<(const Quaternion& q) const {
    return norm() < q.norm();
}

bool Quaternion::operator<=(const Quaternion& q) const {
    float norm_ = norm();
    float q_norm = q.norm();

    return norm_ < q_norm || norm_ == q_norm;
}

// Relaxed comparison function
// Inequalities evaluated based on norm()

int Quaternion::relaxedCompare(const Quaternion& q, float tolerance) const {

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

float Quaternion::norm(void) const {
    float norm_ = 0;
    for (int i = 0; i < 4; i++) {
        norm_ += data_[i] * data_[i];
    }
    return sqrt(norm_);
}

void Quaternion::setVector(const float * vector) {
    memcpy(vector_, vector, 3 * sizeof (float));
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

float* Quaternion::toEulerAngles(void) {
    // TODO: IMPLEMENT
    float *euler = new float[3];

    return euler;
}
