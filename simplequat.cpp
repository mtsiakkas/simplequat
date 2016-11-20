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

    memcpy(data_, q.getData(), 4 * sizeof (float));
    //    const float *qdata = q.getData();
    //
    //    for (int i = 0; i < 4; i++)
    //        data_[i] = qdata[i];
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
    //    for (int i = 0; i < 3; i++)
    //        vector_[i] = vector[i];
}

Quaternion::Quaternion(const float* data, ConstructorOptions opt) {

    data_ = new float[4];
    scalar_ = data_;
    vector_ = data_ + 1;

    switch (opt) {
        case QuaternionData:
            memcpy(data_, data, 4 * sizeof (float));
            //        for (int i = 0; i < 4; i++)
            //            data_[i] = data[i];
            break;
        case EulerAngleData:
        {

            float roll = data[1];
            float pitch = data[2];
            float yaw = data[3];

            Quaternion qRoll(static_cast<float> (cos(roll / 2.0f)), static_cast<float> (sin(roll / 2.0f)), 0.0f, 0.0f);
            Quaternion qPitch(static_cast<float> (cos(pitch / 2.0f)), 0.0f, static_cast<float> (sin(pitch / 2.0f)), 0.0f);
            Quaternion qYaw(static_cast<float> (cos(yaw / 2.0f)), 0.0f, 0.0f, static_cast<float> (sin(yaw / 2.0f)));
            Quaternion tmp = qRoll * qPitch*qYaw;

            for (int i = 0; i < 4; i++)
                data_[i] = tmp.getData()[i];

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

    //    const float *qdata = q.getData();
    //    
    //    for (int i = 0; i < 4; i++)
    //        data_[i] = qdata[i];

    memcpy(data_, q.getData(), 4 * sizeof (float));

    return *this;
}

Quaternion Quaternion::operator+(const Quaternion& q) const {
    float newdata[4];

    for (int i = 0; i < 4; i++)
        newdata[i] = data_[i] + q.getData()[i];

    return Quaternion(newdata);
}

Quaternion Quaternion::operator-(const Quaternion& q) const {
    float newdata[4];

    for (int i = 0; i < 4; i++)
        newdata[i] = data_[i] - q.getData()[i];

    return Quaternion(newdata);
}

// Overload -- Multiplication by scalar

Quaternion Quaternion::operator*(const float gam) const {
    //  if (*this == Quaternion::ZERO) return Quaternion::ZERO;

    float newdata[4];

    for (int i = 0; i < 4; i++)
        newdata[i] = data_[i] * gam;

    return Quaternion(newdata);
}

float* Quaternion::rotate3vector(const float* f) {
    if (isNormalized()) {
        float* out = new float[3];

        Quaternion fo = *this * Quaternion(0, f) * (this->conjugate());

        const float* fov = fo.getVector();
        for (int i = 0; i < 3; i++)
            out[i] = fov[i];

        return out;
    } else {
        throw quat_exc_not_normalized(norm());
    }
}

// Overload -- Division by scalar

Quaternion Quaternion::operator/(const float gam) const {
    if (*this == Quaternion::ZERO)
        return Quaternion::ZERO;

    if (gam == 0) {
        throw quat_exc_division_by_zero();
    } else {
        float qdata[4];

        for (int i = 0; i < 4; i++)
            qdata[i] = data_[i] / gam;

        return Quaternion(qdata);
    }
}

Quaternion& Quaternion::operator+=(const Quaternion& q) {
    for (int i = 0; i < 4; i++)
        data_[i] += q.getData()[i];

    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& q) {
    for (int i = 0; i < 4; i++)
        data_[i] -= q.getData()[i];

    return *this;
}

Quaternion& Quaternion::operator*=(const float gam) {
    for (int i = 0; i < 4; i++)
        data_[i] *= gam;

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
        for (int i = 0; i < 4; i++)
            data_[i] /= gam;
        return *this;
    }
}

Quaternion Quaternion::operator-(void) {
    float qdata[4];

    for (int i = 0; i < 4; i++)
        qdata[i] = -data_[i];

    return Quaternion(qdata);
}

float& Quaternion::operator[](int index) {
    if (index > -1 && index < 4) {
        return data_[index];
    } else {
        throw quat_exc_generic("QUATERNION::INDEX OUT OF RANGE");
    }
}

Quaternion Quaternion::conjugate(void) const {
    float vector[3];

    for (int i = 0; i < 3; i++)
        vector[i] = -vector_[i];

    return Quaternion(*scalar_, vector);
}

Quaternion Quaternion::inverse(void) const {
    return conjugate() / norm();
}

// Overload -- Quaternion product

Quaternion Quaternion::operator*(const Quaternion& q) const {
    if ((*this == Quaternion::ZERO) || (q == Quaternion::ZERO))
        return Quaternion::ZERO;

    if (*this == Quaternion::UNIT)
        return q;

    if (q == Quaternion::UNIT)
        return *this;

    const float* q_vector = q.getVector();
    float q_scalar = q.getScalar();

    float scalar = *scalar_ * q.getData()[0];

    for (int i = 0; i < 3; i++)
        scalar -= q_vector[i] * vector_[i];

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
    return memcmp(data_, q.getData(), 4 * sizeof (float)) == 0;
}

bool Quaternion::operator!=(const Quaternion& q) const {
    return !(*this == q);
}

bool Quaternion::operator>(const Quaternion& q) const {
    return this->norm() > q.norm();
}

bool Quaternion::operator>=(const Quaternion& q) const {
    float norm = this->norm();
    float q_norm = q.norm();

    return norm > q_norm || norm == q_norm;
}

bool Quaternion::operator<(const Quaternion& q) const {
    return this->norm()<q.norm();
}

bool Quaternion::operator<=(const Quaternion& q) const {
    float norm = this->norm();
    float q_norm = q.norm();

    return norm < q_norm || norm == q_norm;
}

int Quaternion::relaxedCompare(const Quaternion& q, float tolerance) const {
    if(tolerance == 0) {
        bool geq = *this>=q;
        bool leq = *this<=q;
        if(geq && leq)
            return 0;
        return geq ? 1 : -1;
    }
    
    return 0;
}


float Quaternion::norm(void) const {
    float norm = 0;
    for (int i = 0; i < 4; i++)
        norm += data_[i] * data_[i];

    return sqrt(norm);
}

void Quaternion::setVector(const float * vector) {
    for (int i = 0; i < 3; i++)
        vector_[i] = vector[i];
}

std::string Quaternion::toString(void) {
    std::stringstream ss;

    ss << "[";
    for (int i = 0; i < 4; i++)
        ss << " " << data_[i];
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
