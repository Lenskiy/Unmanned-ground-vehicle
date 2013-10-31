/*
Description:    This module find the rotational offset required by Gilies Curve.
                Input:  Contour Pointer
                Output: Array of rotational offset

Date Created:   22/11/2011
Date Modified:  27/11/2011
                (Transformation from C style to C++ style. Use of vector instead of structure)
Author:         Team T
*/


#include "RotationalOffset.h"

//Constructor
cRotationalOffset::cRotationalOffset()
{

    //mpOutput.resize(cMAX_POINTS);
}

//Destructor
cRotationalOffset::~cRotationalOffset()
{
}


void cRotationalOffset::CartisanToPolar( const Contour &contour )
{
    //unsigned int i;

    int vXc         = 0;
    int vYc         = 0;
    int vX          = 0;
    int vY          = 0;
    float vRad      = 0.0;
    float vTheta    = 0.0;
    //mNoOfPoints       = contour->nop;
    //vector< pair<float, float> >  CPtr    = contour;

    Contour::const_iterator i;
    //Calculate the Centre of Mass(COM)of the contour.

    for ( i = contour.begin(); i != contour.end(); i++ )
    {
        vXc         += i->first;
        vYc         += i->second;
        //contour    =  contour->next;
        //contour++;//change here...above one is correct
    }

    //Normalise the COM
    vXc /= contour.size();//mNoOfPoints;

    vYc /= contour.size();//mNoOfPoints;

    //Restore the base pointer.
    //contour = CPtr;
    
    //Translation of the origin to COM and convert from Cartisian to Polar co-ordinates
    for ( i = contour.begin(); i != contour.end(); i++ )
    {
        vX       = i->first - vXc;
        vY       = i->second - vYc;
        vRad = static_cast<float>( sqrt( static_cast<double>( vX * vX ) +
                                         static_cast<double>( vY * vY ) ) );

        vTheta = static_cast<float>( atan2( static_cast<double>( vY ),
                                            static_cast<double>( vX ) ) );  /*Gives the output in range -PI to PI */

        //if theta lies in 3rd or 4th quadrant then make it in range of 0 - 2 * pi, add 2*PI to all theta values.

        if ( vY < 0 )
            vTheta += 2 * static_cast<float>( cPI );

        /*Store information<theta,radius> in vector pair*/
        mThetaRad.push_back( make_pair( vTheta, vRad ) );

        //contour = contour->next;
        //contour++;//change here...above one is correct
    }
}


void cRotationalOffset::FindMinimum( void )
{
    float vCurr   = 0;
    float vPrev   = 0;
    float vPrev2  = 0;
    float vPrev8  = 0;
    float vPrev9  = 0;
    float vPrev10 = 0;
    float vPrev3  = 0;
    float vPrev4  = 0;
    float vPrev5  = 0;
    float vPrev6  = 0;
    float vPrev7  = 0;
    float vNext   = 0;
    float vNext3  = 0;
    float vNext5  = 0;
    float vNext7  = 0;
    float vNext9  = 0;
    float vNext2  = 0;
    float vNext4  = 0;
    float vNext6  = 0;
    float vNext8  = 0;
    float vNext10 = 0;


    //Start from 3rd pair
    vector<pair<float, float> >::iterator i;

    //Check till 2nd last element.

    for ( i = mThetaRad.begin() + 10; i != mThetaRad.end() - 10; i++ )
    {
        //Current object
        vCurr = i->second;

        //Previous element
        vPrev = ( i - 1 )->second;

        //Previous to previous element
        vPrev2 = ( i - 2 )->second;

        //Previous to previous element
        vPrev3 = ( i - 3 )->second;

        //Previous to previous element
        vPrev4 = ( i - 4 )->second;

        //Previous to previous element
        vPrev5 = ( i - 5 )->second;

        //Previous to previous element
        vPrev6 = ( i - 6 )->second;

        //Previous to previous element
        vPrev7 = ( i - 7 )->second;

        //Previous to previous element
        vPrev8 = ( i - 8 )->second;

        //Previous to previous element
        vPrev9 = ( i - 9 )->second;

        //Previous to previous element
        vPrev10 = ( i - 10 )->second;

        //Next element
        vNext  = ( i + 1 )->second;

        //Next to next element
        vNext2 = ( i + 2 )->second;

        //Next to next element
        vNext3 = ( i + 3 )->second;

        //Next to next element
        vNext4 = ( i + 4 )->second;

        //Next to next element
        vNext5 = ( i + 5 )->second;

        //Next to next element
        vNext6 = ( i + 6 )->second;

        //Next to next element
        vNext7 = ( i + 7 )->second;

        //Next to next element
        vNext8 = ( i + 8 )->second;

        //Next to next element
        vNext9 = ( i + 9 )->second;

        //Next to next element
        vNext10 = ( i + 10 )->second;

        //if the current point is less tha its 2 neighbouring points, store the corresponding theta in o/p array

        if ( vCurr < vPrev  && vCurr < vPrev2  && vCurr < vNext  && vCurr < vNext2 &&
             vCurr < vPrev3 && vCurr < vPrev4  && vCurr < vNext3 && vCurr < vNext4 &&
             vCurr < vPrev5 && vCurr < vPrev6  && vCurr < vNext5 && vCurr < vNext6 &&
             vCurr < vPrev7 && vCurr < vPrev8  && vCurr < vNext7 && vCurr < vNext8 &&
             vCurr < vPrev9 && vCurr < vPrev10 && vCurr < vNext9 && vCurr < vNext10 )
        {
            //Store the minimum theta in output array
            //mpOutput[count] = i->first;
            mpOutput.push_back( i->first );
            //count++;
        }
    }



}


vector<float> cRotationalOffset::GetMinRadius( const Contour &contour )
{
    Contour::iterator i;

    mpOutput.clear();
    CartisanToPolar( contour );
    /* Sort in pairs ascending order.
    By default sort function sort on the basis of 1st element of pairs.
    Thats exactly we want !! Sort radius on the basis of theta.
    Therefore no need to provide external Compare function. ;)  */
    sort( mThetaRad.begin(), mThetaRad.end() );
    FindMinimum();
    mThetaRad.clear();

    return mpOutput;
}
