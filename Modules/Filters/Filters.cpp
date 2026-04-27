#include "Filters.h"
#include <math.h>
#include <string.h>

using namespace std;

/**
  * @brief Destructor
  * @param None
  * @retval None
  */
PT1Filter::~PT1Filter()
{

}

/**
  * @brief Compute RC filter parameter
  * @param f_cut: cutoff frequency in Hz
  * @retval RC float value
  */
float PT1Filter::ComputeRC(const float f_cut)
{
    return 1.0f / (2.0f * M_PI * f_cut);
}

/**
  * @brief Initialize filter with RC
  * @param tau : circuit time constant in microseconds
  *        dT : time delta, in microseconds, between two measurements
  * @retval None
  */
void PT1Filter::FilterInitRC(float tau, float dT)
{
    ft_state = 0.0f;
    ft_RC = tau;
    ft_dT = dT;
    ft_alpha = ft_dT / (ft_RC + ft_dT);
}

/**
  * @brief Initialize filter
  * @param f_cut: cutoff frequency in Hz
  *        dT : time delta, in microseconds, between two measurements
  * @retval None
  */
void PT1Filter::FilterInit(float f_cut, float dT)
{
    FilterInitRC(ComputeRC(f_cut), dT);
}

/**
  * @brief Set time constant tau. RC=tau
  * @param f_cut: cutoff frequency in Hz
  *        dT : time delta, in microseconds, between two measurements
  * @retval None
  */
void PT1Filter::FilterSetTimeConstant(float tau)
{
    ft_RC = tau;
}

/**
  * @brief Apply filter to the measured value
  * @param input: measured value
  * @retval filtered value
  */
float  PT1Filter::FilterApply(float input)
{
    first_load = false;
    ft_state = ft_state + ft_alpha * (input - ft_state);
    return ft_state;
}

/**
  * @brief Apply filter to the measured value.
  *        Additional dT parameter gives us more room if fT not a constant
  * @param input: measured value
  *        dT: time delta, in microseconds, between two measurements
  * @retval input: measured value
  */
float  PT1Filter::FilterApply(float input, float dT)
{
    first_load = false;
    ft_dT = dT;
    ft_alpha = ft_dT / (ft_RC + ft_dT);
    ft_state = ft_state + ft_alpha * (input - ft_state);
    return ft_state;
}

/**
  * @brief Apply filter to the measured value.
  *        Additional dT parameter gives us more room if fT not a constant
  * @param input: measured value
  *        dT: time delta, in microseconds, between two measurements
  *        RC: circuit time constant
  * @retval input: measured value
  */
float  PT1Filter::FilterApply(float input, float dT, float fcut)
{
    first_load = false;
    if(!ft_RC)
    {
        ft_RC = ComputeRC(fcut);
    }

    ft_dT = dT;
    ft_alpha = ft_dT / (ft_RC + ft_dT);
    ft_state = ft_state + ft_alpha * (input - ft_state);
    return ft_state;
}

/**
  * @brief Set preloaded state value
  * @param val : new state value
  * @retval None
  */
void PT1Filter::FilterSetVal(float val)
{
    ft_state = val;
}


/**
  * @brief Destructor
  * @param None
  * @retval None
  */
FirFilter::~FirFilter()
{

}

/**
  * @brief 
  * @param 
  * @retval None
  */
void FirFilter::firFilterInit(float *buf, uint8_t bufLength, const float *coeffs)
{
    this->buf = buf;
    this->bufLength = bufLength;
    this->coeffs = coeffs;
    coeffsLength = bufLength;
    memset(this->buf, 0, sizeof(float) * this->bufLength);
}

/**
  * @brief 
  * @param 
  * @retval None
  */
void FirFilter::firFilterUpdate(float input)
{
    memmove(&buf[1], &buf[0], (bufLength - 1) * sizeof(float));
    buf[0] = input;
}

/**
  * @brief 
  * @param 
  * @retval None
  */
float FirFilter::firFilterApply()
{
    float ret = 0.0f;

    for (int i = 0; i < coeffsLength; ++i) 
    {
        ret += coeffs[i] * buf[i];
    }

    return ret;
}