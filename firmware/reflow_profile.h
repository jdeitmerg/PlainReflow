/* This file is part of PlainReflow, a simple reflow oven controller.
 *
 * @file reflow_profile.h
 * @author Jonas Deitmerg
 * @date 2017
 * @copyright MIT License
 *
 * For more information, see https://github.com/jdeitmerg/PlainReflow
 */

class reflow_profile {
    public:
        /* The times and temperatures arrays are not copied to internal
         * arrays. Instead, only their reference is stored. Make sure the
         * arrays you pass are statically allocated and accessible
         * during all operations of this class's functions.
         *
         * times are in ms since restart was called, temperatures are in °C.
         */
        reflow_profile(unsigned int numvals, const unsigned long times[],
                       const float temperatures[]);
        // Reset/start reflow profile
        void reset(void);
        float getval(void);
    private:
        unsigned long int reset_timestamp;
        unsigned int nvals;
        const unsigned long * timestamps;
        const float * tempvals;
};

