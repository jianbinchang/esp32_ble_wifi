/*!
///////////////////////////////////////////////////////////
 *  \file      jcc_ellipsoid.h
 *  \brief     jcc ellipsoid
 *  \details   Implementation ellipsoid fit
 *  \author    张永强
 *  \version   0.1
 *  \date      23-09-2016 14:57
///////////////////////////////////////////////////////////
*/
#if !defined(JCCFIT__INCLUDED_)
#define JCCFIT__INCLUDED_


#ifdef __cplusplus
extern "C" {
#endif

void ellipsoid_reset(void);
void ellipsoid_rawdata(float x , float y , float z);
void ellipsoid_process(float radius,float *sx,float *sy,float *sz,float *ox,float *oy,float *oz);

#ifdef __cplusplus
}
#endif

#endif
