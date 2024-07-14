/**
 * @file lowpassfilter2p.h
 * @author Haojia Li (hlied@connect.ust.hk)
 * @brief low pass filter 
 * @version 1.0
 * @date 2022-10-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
// namespace PayloadMPC
// {
    template <typename T>
    class LowPassFilter2p
    {
    public:
        LowPassFilter2p() = default;

        LowPassFilter2p(double sample_freq, double cutoff_freq)
        {
            // set initial parameters
            set_cutoff_frequency(sample_freq, cutoff_freq);
        }

        // Change filter parameters
        void set_cutoff_frequency(double sample_freq, double cutoff_freq)
        {
            if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2) || !std::isfinite(sample_freq) || !std::isfinite(cutoff_freq))
            {

                disable();
                return;
            }

            // reset delay elements on filter change
            _delay_element_1 = {};
            _delay_element_2 = {};

            _cutoff_freq = std::max(cutoff_freq, sample_freq * 0.001f);
            _sample_freq = sample_freq;

            const double fr = _sample_freq / _cutoff_freq;
            const double ohm = tan(M_PI / fr);
            const double c = 1.f + 2.f * cosf(M_PI / 4.f) * ohm + ohm * ohm;

            _b0 = ohm * ohm / c;
            _b1 = 2.f * _b0;
            _b2 = _b0;

            _a1 = 2.f * (ohm * ohm - 1.f) / c;
            _a2 = (1.f - 2.f * cos(M_PI / 4.f) * ohm + ohm * ohm) / c;

            if (!std::isfinite(_b0) || !std::isfinite(_b1) || !std::isfinite(_b2) || !std::isfinite(_a1) || !std::isfinite(_a2))
            {
                disable();
            }
        }

        /**
         * Add a new raw value to the filter
         *
         * @return retrieve the filtered result
         */
        inline T apply(const T &sample)
        {
            // Direct Form II implementation
            T delay_element_0{sample - _delay_element_1 * _a1 - _delay_element_2 * _a2};

            const T output{delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2};

            _delay_element_2 = _delay_element_1;
            _delay_element_1 = delay_element_0;

            return output;
        }

        // Filter array of samples in place using the Direct form II.
        inline void applyArray(T samples[], int num_samples)
        {
            for (int n = 0; n < num_samples; n++)
            {
                samples[n] = apply(samples[n]);
            }
        }

        // Return the cutoff frequency
        double get_cutoff_freq() const { return _cutoff_freq; }

        // Return the sample frequency
        double get_sample_freq() const { return _sample_freq; }

        double getMagnitudeResponse(double frequency) const;

        // Reset the filter state to this value
        T reset(const T &sample)
        {
            // const T input = std::isfinite(sample) ? sample : T{};
            const T input = sample;

            if (fabs(1 + _a1 + _a2) > 1.0e-8)
            {
                _delay_element_1 = _delay_element_2 = input / (1 + _a1 + _a2);
            }
            else
            {
                _delay_element_1 = _delay_element_2 = input;
            }

            return apply(input);
        }

        void disable()
        {
            // no filtering
            _sample_freq = 0.f;
            _cutoff_freq = 0.f;

            _delay_element_1 = {};
            _delay_element_2 = {};

            _b0 = 1.f;
            _b1 = 0.f;
            _b2 = 0.f;

            _a1 = 0.f;
            _a2 = 0.f;
        }

    protected:
        T _delay_element_1{}; // buffered sample -1
        T _delay_element_2{}; // buffered sample -2

        // All the coefficients are normalized by a0, so a0 becomes 1 here
        double _a1{0.f};
        double _a2{0.f};

        double _b0{1.f};
        double _b1{0.f};
        double _b2{0.f};

        double _cutoff_freq{0.f};
        double _sample_freq{0.f};
    };
// }