// HEADER FILE FOR DH2LINK.CPP

#ifndef DH2LINK_HPP
#define DH2LINK_HPP

class dh2link
{
    private:
    // Attributes
    float a_1,
          alpha_1,
          d_1,
          theta_1,
          q_1;
    //
    float a_2, 
          alpha_2,
          d_2,
          theta_2,
          q_2;
    //
    float x_pos,
          y_pos,
          z_pos;
    // Utility
    void calculate();

    public:
    // set_ methods
    void setLink1(float a_1, float alpha_1, float d_1, float tetha_1),
         setLink2(float a_2, float alpha_2, float d_2, float tetha_2),
         setQ1(float q_1),
         setQ2(float q_2),
         setQs(float q_1, float q_2);
    // get_ methods
    float getX(),
          getY(),
          getZ();
    // Other methods
    void printData() const; // Prints attributes
    
    // Default (unparametrized) constructor
    dh2link();
    // Parametrized constructor
    dh2link(float a_1, float alpha_1, float d_1, float tetha_1,
            float a_2, float alpha_2, float d_2, float tetha_2);
    // Destructor
    ~dh2link();
};

#endif