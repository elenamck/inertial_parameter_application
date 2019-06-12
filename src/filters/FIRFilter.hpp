//
//  FIRFilter.hpp
//  am2b
//
//  Created by Felix Sygulla on 2018-05-30.
//  Copyright 2018 Chair of Applied Mechanics, TUM. All rights reserved.
//

// #include <vector>

// #pragma once


#include <iostream>
#include <stdexcept>

namespace am2b {

template <typename T>
class FIRFilter {

public:
    //! default constructor
    FIRFilter() {}

    //! Prototype initialization constructor for complex data types
    //! This prototype should be equal "zero", as it is used for reinitialization
    FIRFilter(const T& prototype)
        : m_value(prototype)
        , m_prototype(prototype)
    {
    }

    /* Init the FIR filter
      \param filterLength Number of filter coefficients (= Filter Order + 1)
      \param filterCoefficients FIR coefficients array. Start with the coefficient for the newest data sample
    */
    void init(const std::size_t& filterLength, const double* const filterCoefficients)
    {
        m_filterLength = filterLength;
        m_coeffs = filterCoefficients;
        m_buffer.clear();
        m_buffer.reserve(filterLength);
    }

    //! Reset the FIR filter value to a prototype
    void reset(const T& prototype)
    {
        m_value = prototype;
        m_filterIndex = 0;
    }

    //! update the filter value with new sample data
    const T& update(const T& sample)
    {
        // Uses a combination of a ring buffer and
        // an aggregated value to have maximum speed
        // (no need to execute all multiplications every step)
        if (m_buffer.size() < m_filterLength) {
            m_buffer.push_back(sample);
        } else {
            m_buffer[m_filterIndex] = sample;
        }

        // increase index (then points to the oldest value)
        m_filterIndex++;
        if (m_filterIndex > m_filterLength - 1) {
            m_filterIndex = 0;
        }

        m_value = m_prototype;

        // Calculate the filter output
        // coeffIdx wraps around the array to simulate the shifting of
        // data in the vector without actually doing so
        for (std::size_t i = 0; i < m_filterLength; i++) {

            std::size_t coeffIdx = (m_filterIndex + i) % m_filterLength;

            // in case the filter is not settled yet
            if (coeffIdx >= m_buffer.size() && m_buffer.size() != m_filterLength) {
                continue;
            }

            m_value += m_buffer[coeffIdx] * m_coeffs[m_filterLength - 1 - i];
        }

        return m_value;
    }

    //! Returns the filter output
    const T& getOutput()
    {
        return m_value;
    }

private:
    //! Length of the circular buffer used for the filter
    std::size_t m_filterLength = 0;

    //! Filter coefficient index for the next sample
    std::size_t m_filterIndex = 0;

    //! Current filter value
    T m_value = 0;

    //! Zero Prototype value
    T m_prototype = 0;

    //! Pointer to filter coefficients
    const double* m_coeffs;

    //! The ring buffer for all sampled values of the given order
    std::vector<T> m_buffer;
};
}
