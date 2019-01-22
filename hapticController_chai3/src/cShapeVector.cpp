//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2010 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.2.0 $Rev: 598 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "cShapeVector.h"
//#include "graphics/CMacrosGL.h"
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif

#include "graphics/CDraw3D.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cShapeVector.

    \fn     cShapeVector::cShapeVector(const cVector3d& a_pointA, const cVector3d& a_pointB)
	\param	a_pointA  Point A of line.
	\param	a_pointB  Point B of line.
*/
//===========================================================================
cShapeVector::cShapeVector(const cVector3d& a_pointA, const cVector3d& a_pointB, double a_headSize, double a_lineWidth)
{
    // initialize line with start and end points.
    m_pointA.copyfrom(a_pointA);
    m_pointB.copyfrom(a_pointB);

    m_headSize = a_headSize;
    m_lineWidth = a_lineWidth;

    // set color properties
    m_colorPointA.set(1.0, 1.0, 1.0, 1.0);
    m_colorPointB.set(1.0, 1.0, 1.0, 1.0);
};


//===========================================================================
/*!
    Render sphere in OpenGL

    \fn       void cShapeVector::render(const int a_renderMode)
    \param    a_renderMode  See cGenericObject::render()
*/
//===========================================================================
void cShapeVector::render(cRenderOptions &a_options)
{
    //-----------------------------------------------------------------------
    // Conditions for object to be rendered
    //-----------------------------------------------------------------------

    if(a_options.m_render_transparent_front_faces_only ||
       a_options.m_render_transparent_back_faces_only)
    {
        return;
    }

    //-----------------------------------------------------------------------
    // Rendering code here
    //-----------------------------------------------------------------------

    glDisable(GL_LIGHTING);

    #define ARROW_CYLINDER_PORTION 0.8
    #define ARRROW_CONE_PORTION (1.0 - ARROW_CYLINDER_PORTION)

    cVector3d lineEnd = m_pointA;
    lineEnd.add(cSub(m_pointB, m_pointA) * ARROW_CYLINDER_PORTION);

    double p1, p2;

    p1 = m_pointA.x();
    p2 = lineEnd.x();

    // draw line
    glLineWidth(m_lineWidth);
    glBegin(GL_LINES);
        m_colorPointA.render();
        glVertex3dv(&p1);
        m_colorPointB.render();
        glVertex3dv(&p2);
    glEnd();


  return;
    glPushMatrix();

    // We don't really care about the up vector, but it can't
    // be parallel to the arrow...
    cVector3d up = cVector3d(0,1,0);
    cVector3d arrow = m_pointB-m_pointA;
    arrow.normalize();
    double d = fabs(cDot(up,arrow));
    if (d > .9)
    {
        up = cVector3d(1,0,0);
    }

    cLookAt(m_pointA, m_pointB, up);
    double distance = cDistance(m_pointB,m_pointA);

    // This flips the z axis around
    glRotatef(180,1,0,0);

    // create a new OpenGL quadratic object
    GLUquadricObj *quadObj;
    quadObj = gluNewQuadric();

    // set rendering style
    gluQuadricDrawStyle(quadObj, GLU_FILL);

    // set normal-rendering mode
    gluQuadricNormals(quadObj, GLU_SMOOTH);

    // render a cylinder and a cone
    glRotatef(180,1,0,0);
    //gluDisk(quadObj,0,m_lineWidth,10,10);
    glRotatef(180,1,0,0);

    //gluCylinder(quadObj, m_lineWidth, m_lineWidth, distance*ARROW_CYLINDER_PORTION, 10, 10);
    glTranslated(0, 0, ARROW_CYLINDER_PORTION*distance);

    glRotatef(180, 1, 0, 0);
    m_colorPointB.render();
    gluDisk(quadObj, 0, m_headSize, 10, 10);
    glRotatef(180,1,0,0);

    m_colorPointB.render();
    gluCylinder(quadObj, m_headSize, 0.0, distance*ARRROW_CONE_PORTION, 30, 30);

    // delete our quadric object
    gluDeleteQuadric(quadObj);

    glPopMatrix();
    glEnable(GL_LIGHTING);

}

//===========================================================================
/*!
    From the position of the tool, search for the nearest point located
    at the surface of the current object. Decide if the point is located inside
    or outside of the object

    \fn     void cShapeVector::computeLocalInteraction(const cVector3d& a_toolPos,
                                                      const cVector3d& a_toolVel,
                                                      const unsigned int a_IDN)
    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void cShapeVector::computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN)
{
    // the tool can never be inside the line
    m_interactionInside = false;

    // if both point are equal
    m_interactionPoint = cProjectPointOnSegment(a_toolPos,
                                                m_pointA,
                                                m_pointB);
}


//===========================================================================
/*!
    Update bounding box of current object.

    \fn       void cShapeVector::updateBoundaryBox()
*/
//===========================================================================
void cShapeVector::updateBoundaryBox()
{
    m_boundaryBoxMin.set(cMin(m_pointA.x(), m_pointB.x()),
                         cMin(m_pointA.y(), m_pointB.y()),
                         cMin(m_pointA.z(), m_pointB.z()));

    m_boundaryBoxMax.set(cMax(m_pointA.x(), m_pointB.x()),
                         cMax(m_pointA.y(), m_pointB.y()),
                         cMax(m_pointA.z(), m_pointB.z()));
}


//===========================================================================
/*!
    Scale object of defined scale factor

    \fn       void cShapeVector::scaleObject(const cVector3d& a_scaleFactors)
    \param    a_scaleFactors Scale factor
*/
//===========================================================================
void cShapeVector::scaleObject(const cVector3d& a_scaleFactors)
{
    m_pointA.x(a_scaleFactors.x() * m_pointA.x());
    m_pointA.y(a_scaleFactors.y() * m_pointA.y());
    m_pointA.z(a_scaleFactors.z() * m_pointA.z());

    m_pointB.x(a_scaleFactors.x() * m_pointB.x());
    m_pointB.y(a_scaleFactors.y() * m_pointB.y());
    m_pointB.z(a_scaleFactors.z() * m_pointB.z());
}
