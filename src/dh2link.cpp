// CODE INTEDED TO CALCULATE X,Y,Z COORDINATES FOR A TWO LINK MODEL
// D-H PARAMETERS MUST BE PREVIOUSLY CALCULATED

#include "dh2link.hpp"
#include <iostream>
#include <math.h>

#define PI 3.14159265

// CONSTRUCTORS

dh2link::dh2link()
{
    //
    this->a_1 = 0;
    this->alpha_1 = 0;
    this->d_1 = 0;
    this->theta_1 = 0;
    this->q_1 = 0;
    //
    this->a_2 = 0;
    this->alpha_2 = 0;
    this->d_2 = 0;
    this->theta_2 = 0;
    this->q_2 = 0;
    //
    this->x_pos = 0;
    this->y_pos = 0;
    this->z_pos = 0;
}

dh2link::dh2link(float a_1, float alpha_1, float d_1, float tetha_1,
                 float a_2, float alpha_2, float d_2, float tetha_2)
{
    //
    this->a_1 = a_1;
    this->alpha_1 = alpha_1*PI/180;
    this->d_1 = d_1;
    this->theta_1 = tetha_1*PI/180;
    this->q_1 = 0;
    //
    this->a_2 = a_2;
    this->alpha_2 = alpha_2*PI/180;
    this->d_2 = d_2;
    this->theta_2 = tetha_2*PI/180;
    this->q_2 = 0;
    //
    this->x_pos = 0;
    this->y_pos = 0;
    this->z_pos = 0;
}

// DESTROYER

dh2link::~dh2link()
{
    std::cout << "Instanced destroyed.\n";
}

// UTILITY

void dh2link::calculate() // This function was determined by linear algebra in order to prevent working with matrices
{
    this->x_pos = cos(this->theta_1+this->q_1)*this->a_1*cos(this->theta_2+this->q_2) - sin(this->theta_1+this->q_1)*cos(this->alpha_1)*this->a_2*sin(this->theta_2+this->q_2) + sin(this->theta_1+this->q_1)*sin(this->alpha_1)*this->d_2 + this->a_1*cos(this->theta_1+this->q_1)*1;
    this->y_pos = sin(this->theta_1+this->q_1)*this->a_1*cos(this->theta_2+this->q_2) + cos(this->theta_1+this->q_1)*cos(this->alpha_1)*this->a_2*sin(this->theta_2+this->q_2) - cos(this->theta_1+this->q_1)*sin(this->alpha_1)*this->d_2 + this->a_1*sin(this->theta_1+this->q_1)*1;
    this->z_pos =                                                   sin(this->alpha_1)                   *this->a_2*sin(this->theta_2+this->q_2) + cos(this->alpha_1)                   *this->d_2 + this->d_2                   *1;
}

// SETTERS

void dh2link::setLink1(float a_1, float alpha_1, float d_1, float tetha_1)
{
    this->a_1 = a_1;
    this->alpha_1 = alpha_1*PI/180;
    this->d_1 = d_1;
    this->theta_1 = tetha_1*PI/180;
}

void dh2link::setLink2(float a_2, float alpha_2, float d_2, float tetha_2)
{
    this->a_2 = a_2;
    this->alpha_2 = alpha_2*PI/180;
    this->d_2 = d_2;
    this->theta_2 = tetha_2*PI/180;
}

void dh2link::setQ1(float q_1) //
{
    this->q_1 = q_1;
}

void dh2link::setQ2(float q_2) //
{
    this->q_2 = q_2;
}

void dh2link::setQs(float q_1, float q_2) //
{
    this->q_1 = q_1;
    this->q_2 = q_2;
}

// GETTERS

float dh2link::getX()
{
    dh2link::calculate();
    return this->x_pos;
}

float dh2link::getY()
{
    dh2link::calculate();
    return this->y_pos;
}

float dh2link::getZ()
{
    dh2link::calculate();
    return this->z_pos;
}

// PRINTs

void dh2link::printData() const
{
    std::cout <<"a_1 = "<<this->a_1<<" | alpha_1 = "<<this->alpha_1<<" | d_1 = "<<this->d_1<<" | theta_1 = "<<this->theta_1<<"\n"
              <<"a_2 = "<<this->a_2<<" | alpha_2 = "<<this->alpha_2<<" | d_2 = "<<this->d_2<<" | theta_2 = "<<this->theta_2<<"\n"
              <<"All units are milimiters and radians. \n";
}

//