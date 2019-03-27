//
//  SecOrderLowPass.hpp
//  am2b
//
//  Created by Felix Sygulla on 2018-02-21.
//  Copyright 2018 Chair of Applied Mechanics, TUM. All rights reserved.
//

#pragma once

#include <xdebug.h>
#include <xstdio.h>

namespace am2b {

/*! Second order digital low-pass filter (PT2) for custom data type */
template <typename T>
class SecOrderLowPass {

public:
    /*! Constructor
        \param _initialValue Initial value or template object
    */
    SecOrderLowPass(const T& _initialValue)
        : m_x(_initialValue)
        , m_x1(_initialValue)
        , m_x2(_initialValue)
        , m_u1(_initialValue)
        , m_u2(_initialValue)
    {
    }

    /*! (Re) initialize the filter parameters
        \param _dt sample time
        \param _T time constant of the filter (set to zero to deactivate filter)
        \param _d damping parameter (0.0 to 1.0)
    */
    void init(const float& _dt, const float& _T, const float& _d = 1.0)
    {
        m_T = _T;
        m_dt = _dt;
        m_d = _d;

        if (m_T == 0) {
            // passthru
            m_a0 = 1.0;
            m_a1 = 0;
            m_a2 = 0;
            m_a3 = 1.0;
            m_a4 = 0.0;
            m_a5 = 0.0;
        } else {

            // Stability margin
            if (m_T < m_dt / 2) {
                perr_ffl("Time constant lower than half of sample time, saturating value...\n");
                m_T = m_dt / 2;
            }

            // calculate filter parameters
            m_a0 = 4 * m_T * m_T + 4 * m_d * m_T * m_dt + m_dt * m_dt;
            m_a1 = -2 * m_dt * m_dt + 8 * m_T * m_T;
            m_a2 = -4 * m_T * m_T + 4 * m_d * m_T * m_dt - m_dt * m_dt;
            m_a3 = m_dt * m_dt;
            m_a4 = 2 * m_dt * m_dt;
            m_a5 = m_a3;
        }
    }

    /*! Reset all filter states to given value
        \param _value filter value
    */
    void reset(const T& _value)
    {
        m_x2 = _value;
        m_x1 = _value;
        m_x = _value;
        m_u1 = _value;
        m_u2 = _value;
    }

    /*! Reset filter value to given value and signal rate
        \param _value filter value
        \param _rate filter signal rate
    */
    void reset(const T& _value, const T& _rate)
    {
        m_x = _value;
        m_x1 = m_x - _rate * m_dt;
        m_x2 = m_x1;
        m_u1 = _value;
        m_u2 = _value;
    }

    /*! Compute and return the filter output
        \param _input Filter input value
    */
    const T& process(const T& _input)
    {
        m_x2 = m_x1;
        m_x1 = m_x;

        // update filter output
        m_x = (m_x1 * m_a1 + m_x2 * m_a2 + _input * m_a3 + m_u1 * m_a4 + m_u2 * m_a5) * (1.0 / m_a0);

        m_u2 = m_u1;
        m_u1 = _input;

        return m_x;
    }

    //! Return the current filter output
    const T& getOutput()
    {
        return m_x;
    }

    //! Return the current derivative of the filter output
    T getDerivative()
    {
        return (m_x - m_x1) * (1.0 / m_dt);
    }

    //! Returns the second derivative of the filter output
    T getSecondDerivative()
    {
        return (m_x - 2.0 * m_x1 + m_x2) * (1.0 / m_dt / m_dt);
    }

private:
    //! state
    T m_x;

    //! old state
    T m_x1;

    //! older state
    T m_x2;

    //! old u
    T m_u1;

    //! older u
    T m_u2;

    //! sample time
    float m_dt = 0.0;

    //! time constant
    float m_T = 0.0;

    //! damping
    float m_d = 1.0;

    //! filter coefficient denominator
    float m_a0 = 1.0;

    //! filter coefficient for y[k-1]
    float m_a1 = 0.0;

    //! filter coefficient for y[k-2]
    float m_a2 = 0.0;

    //! filter coefficient for u[k]
    float m_a3 = 0.0;

    //! filter coefficient for u[k-1]
    float m_a4 = 0.0;

    //! filter coefficient for u[k-2]
    float m_a5 = 0.0;
};
} // namespace am2b
