
#ifndef FILTERS_H_
#define FILTERS_H_

class PT1Filter {
public:
    PT1Filter()
    {

    }

    virtual ~PT1Filter();


    bool isFirstLoad() { return first_load; }
    void FilterInit(float f_cut, float dT);
    void FilterInitRC(float tau, float dT);
    void FilterSetTimeConstant(float tau);
    void FilterSetVal(float val);

    float FilterApply(float input);
    float FilterApply(float input, float dT);
    float FilterApply(float input, float dT, float RC);

private:
    bool first_load = true;

    float ft_state = 0;
    float ft_RC = 0;
    float ft_dT = 0;
    float ft_alpha = 0;

    float ComputeRC(const float f_cut);
};

#endif /* FILTERS_H_ */
