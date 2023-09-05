/*!
///////////////////////////////////////////////////////////
 *  \file      jcc_ellipse.h
 *  \brief     jcc ellipse
 *  \details   Implementation ellipse fit
 *  \author    张永强
 *  \version   0.1
 *  \date      23-09-2016 14:57
///////////////////////////////////////////////////////////
*/
#if !defined(JCCFIT2__INCLUDED_)
#define JCCFIT2__INCLUDED_


#ifdef __cplusplus
extern "C" {
#endif

void ellipse_reset(void);
void ellipse_rawdata(float x , float y);
void ellipse_process(float radius,float *sx,float *sy,float *ox,float *oy);

#ifdef __cplusplus
}
#endif

#endif
