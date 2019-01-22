#ifndef CPlane3dH
#define CPlane3dH

#include <vector>
#include <math.h>
#include <limits>
#include "math/CVector3d.h"
#include "world/CMesh.h"
#include "math/CConstants.h"

using namespace chai3d;

#define DOUBLE_MAX std::numeric_limits<double>::max()

// from chai 2.3.0

//! Zero vector (0,0,0)
#define CHAI_VECTOR_ZERO        cVector3d(0, 0, 0)

//! Unit vector along Axis X (1,0,0)
#define CHAI_VECTOR_X           cVector3d(1, 0, 0)

//! Unit vector along Axis Y (0,1,0)
#define CHAI_VECTOR_Y           cVector3d(0, 1, 0)

//! Unit vector along Axis Z (0,0,1)
#define CHAI_VECTOR_Z           cVector3d(0, 0, 1)

//! Origin (0,0,0)
#define CHAI_ORIGIN             cVector3d(0, 0, 0)

// function declarations from geometry.cpp
void decomposeVector3d(const cVector3d& a_this, const cVector3d& a_input, cVector3d& a_parallel, cVector3d& a_perpendicular);
void printVector(const cVector3d& a_this, const unsigned int a_precision=2);

// from chai 2.3
//===========================================================================
/*!
    \struct     cRay3d
    \ingroup    math

    \brief
    cRay3d represents a 3D vector with an origin.
*/
//===========================================================================
struct cRay3d
{
    //! Vector representing ray origin.
    cVector3d m_origin;

    //! Unit vector representing ray direction.
    cVector3d m_direction;

    //! Constructor of cRay3d.
    cRay3d() {}

    //! This constructor assumes that a_direction is normalized already.
    cRay3d(const cVector3d& a_origin, const cVector3d& a_direction) :
        m_origin(a_origin), m_direction(a_direction)
        {  }
};

// represents a plane through a particular point (default is origin)
class cPlane3d
{
    public:
    // CAUTION!! If you modify these properties, call update!

    // MEMBERS:
    // will be modified by update to orthogonalize and normalize
    cVector3d vec1;
    cVector3d vec2;

    cVector3d thru;

    // computed via update
    cVector3d normal;
    double p; // distance of plane from origin along normal

    // CONSTRUCTOR
    cPlane3d() {}

    // construct a plane spanned by vec1 and vec2
    cPlane3d(const cVector3d &_vec1, const cVector3d &_vec2, const cVector3d &_thru = CHAI_ORIGIN) :
        vec1(_vec1), vec2 (_vec2), thru(_thru)
    {
        this->update();
    }

    void copyfrom(const cPlane3d &src) {
        vec1.copyfrom(src.vec1);
        vec2.copyfrom(src.vec2);
        thru.copyfrom(src.thru);
        this->update();
    }

    void update() {
        // normalize vec1
        vec1.normalize();

        // orthogonalize vec2
        //cVector3d vec2perp = cVector3d();
        //cVector3d vec2par = cVector3d();
        //vec2.decompose(vec1, vec2par, vec2perp);

        // normalize vec2
        //vec2perp.normalize();
        //vec2 = vec2perp;
        vec2.normalize();

        // compute normal
        vec1.crossr(vec2, normal);

        // compute distance from origin along normal, store in p
        p = normal.dot(thru);
    }

    inline void getNormal(cVector3d &vec) const {
        vec.copyfrom(normal);
    }

    // return a perpendicular plane spanning vec1 and normal, such that vec2 is its normal
    inline void getPerpendicularPlaneAlongVec1(cPlane3d &plane) const {
        cVector3d invNormal = cVector3d(normal);
        invNormal.negate();
        plane.vec1 = vec1;
        plane.vec2 = invNormal;
        plane.thru = thru;
        plane.update();
    }

    inline void getPerpendicularPlaneAlongVec2(cPlane3d &plane) const {
        cVector3d invNormal = cVector3d(normal);
        invNormal.negate();
        plane.vec1 = invNormal;
        plane.vec2 = vec2;
        plane.thru = thru;
        plane.update();
    }

    // negate normal by negating vec2
    inline void invert() {
        vec2.negate();
        this->update();
    }

    inline void invertr(cPlane3d &dest) const {
        dest.copyfrom(*this);
        dest.invert();
    }

    inline void decomposeVector(const cVector3d &input, cVector3d &par, cVector3d &perp) const {
        perp = normal;
        perp.mul(normal.dot(input));
        par = input - perp;
    }

    inline double distanceToPointSigned(const cVector3d &pt) const {
        cVector3d vec;
        this->projectIntoCoords(pt, vec);
        return vec.z();
    }

    inline void vectorFromPoint(const cVector3d &pt, cVector3d &vec) const {
        vec = normal * -(this->distanceToPointSigned(pt));
    }

    inline void rayToPoint(const cVector3d &pt, cRay3d &ray) const {
        this->vectorFromPoint(pt, ray.m_direction);
        ray.m_direction.negate();
        ray.m_origin = pt - ray.m_direction;
    }

    // translate plane by vector
    inline void translate(const cVector3d &offset) {
        thru += offset;
        this->update();
    }

    inline void translateAlongNormal(double distance) {
        cVector3d offset = cVector3d(normal);
        offset.mul(distance);
        this->translate(offset);
    }

    // convert a point x,y,z into x,y along plane and z off plane
    inline void projectIntoCoords(const cVector3d &pt, cVector3d &coords) const {
        cVector3d inPlane, alongNormal;
        double x,y,z;


        cVector3d deltaPt = cVector3d(pt);
        deltaPt.sub(thru);
        // decompose into along plane and perpendicular to plane
        this->decomposeVector(deltaPt, inPlane, alongNormal);


        // decompose into along plane 1 and along plane 2
        cVector3d copyVec1, alongVec1, alongVec2;
        copyVec1.copyfrom(vec1);
        decomposeVector3d(inPlane, vec1, alongVec1, alongVec2);

        x = alongVec1.dot(vec1);
        y = alongVec2.dot(vec2);
        z = alongNormal.dot(normal);

        coords.set(x, y, z);

        /*
        printf("pt orig ");
        pt.print();
        printf("my thru ");
        thru.print();
        printf("delta ");
        deltaPt.print();
        printf("my vecs\n");
        vec1.print();
        vec2.print();
        normal.print();
        printf("along normal");
        alongNormal.print();
        printf("along plane ");
        inPlane.print();
        printf("along vecs1 2 :\n");
        alongVec1.print();
        alongVec2.print();
        printf("coords ");
        coords.print();
        */

    }

    // rotate a vector direction into plane relative vector coords
    inline void projectDirectionIntoCoords(const cVector3d &dir, cVector3d &coords) const {
        this->projectIntoCoords(cAdd(thru, dir), coords);
    }

    // convert a point in space in plane coordinates into world coordinates
    // inverts projectIntoCoords
    inline void pointFromCoords(const cVector3d &coords, cVector3d &pt) const {
        pt.copyfrom(thru);
        pt += vec1 * coords.x();
        pt += vec2 * coords.y();
        pt += normal * coords.z();
    }

    // convert a direction in space in plane coordinates into world coordinates
    // inverts projectDirectionIntoCoords
    inline void directionFromCoords(const cVector3d &coords, cVector3d &dir) const {
        this->pointFromCoords(coords, dir);
        dir.sub(thru);
    }
};

class cBoundedPlane3d : public cPlane3d
{
    public:
    double halfX; // size along vec1
    double halfY; // size along vec2

    cBoundedPlane3d() : cPlane3d() {}

    cBoundedPlane3d(const cVector3d &_vec1, const cVector3d &_vec2, const cVector3d &_thru,
        double _halfX, double _halfY) :
        cPlane3d(_vec1, _vec2, _thru), halfX(_halfX), halfY(_halfY) {}

    void copyfrom(const cBoundedPlane3d &src) {
        cPlane3d::copyfrom(src);
        halfX = src.halfX;
        halfY = src.halfY;
    }

    inline void halfVectors(cVector3d &hv1, cVector3d &hv2) const
    {
        vec1.normalizer(hv1);
        hv1.mul(halfX);
        vec2.normalizer(hv2);
        hv2.mul(halfY);
    }

    inline void boundedClosestPoint(const cVector3d &pt, cVector3d &closest) const {
        cVector3d coords;
        this->projectIntoCoords(pt, coords);

        // constrain to lie in bounding rect
        if(coords.x() < -halfX)
            coords.x(-halfX);
        if(coords.x() >  halfX)
            coords.x(halfX);
        if(coords.y() < -halfY)
            coords.y(-halfY);
        if(coords.y() >  halfY)
            coords.y(halfY);

        // and lie on plane
        coords.z(0);

        this->pointFromCoords(coords, closest);
    }

    // returns the distance squared to the workspace edge. If inside workspace bounds, distance is positive, if outside, distance is negative
    inline double signedDistanceSquaredFromEdge(const cVector3d &pt) const {
        cVector3d coords;
        this->projectIntoCoords(pt, coords);

        bool outsideX = !cContains(coords.x(), -halfX, halfX);
        bool outsideY = !cContains(coords.y(), -halfY, halfY);
        double dx = cMin(cAbs(coords.x() - halfX), cAbs(coords.x() - -halfX));
        double dy = cMin(cAbs(coords.y() - halfY), cAbs(coords.y() - -halfY));
        if(outsideX)
            if(outsideY)
                return -(dx*dx+dy*dy);
            else
                return -dx*dx;
        else
            if(outsideY)
                return -dy*dy;
            else
                return cMin(dx, dy);
    }

    // get a ray pointing from a point on this bounded plane to the point pt
    // also returns bounded distance signed as below
    inline double boundedRayToPoint(const cVector3d &pt, cRay3d &ray) const {
        cVector3d closest;
        this->boundedClosestPoint(pt, closest);

        ray.m_origin = closest;
        ray.m_direction = pt - closest;

        double boundedDistance = ray.m_direction.length();
        double signedDistance = this->distanceToPointSigned(pt);

        if(signedDistance < 0)
            boundedDistance = -boundedDistance;

        return boundedDistance;
    }

    // return the distance from this bounded plane to the point, negative if outside
    inline double boundedDistanceToPointSigned(const cVector3d &pt) const {
        cVector3d closest, vec;
        this->boundedClosestPoint(pt, closest);
        vec = pt - closest;
        double boundedDistance = vec.length();
        double signedDistance = this->distanceToPointSigned(pt);

        if(signedDistance < 0)
            boundedDistance = -boundedDistance;

        return boundedDistance;
    }

    inline void corners(cVector3d corners[4]) const {
        cVector3d coords;

        // top right
        coords.set(halfX,halfY,0);
        this->pointFromCoords(coords, corners[0]);

        // top left
        coords.set(-halfX,halfY,0);
        this->pointFromCoords(coords, corners[1]);

        // bottom left
        coords.set(-halfX,-halfY,0);
        this->pointFromCoords(coords, corners[2]);

        // bottom right
        coords.set(halfX,-halfY,0);
        this->pointFromCoords(coords, corners[3]);
    }

    inline void printCorners() const {
        cVector3d corners[4];
        this->corners(corners);
        for(int i = 0; i < 4; i++)
            printVector(corners[i]);
    }

    void addToMesh(cMesh* mesh) const {
        //printf("Adding plane to mesh:");
        //this->printCorners();

        int vertices[4];
        cVector3d corners[4];
        this->corners(corners);

        // add vertices to mesh
        for(int i = 0; i < 4; i++)
            vertices[i] = mesh->newVertex(corners[i]);

        // then add triangles
        mesh->newTriangle(vertices[0], vertices[1], vertices[2]);
        mesh->newTriangle(vertices[0], vertices[2], vertices[3]);
    }

};

typedef std::vector<cBoundedPlane3d> BoundedPlaneVector;
class cPlaneBoundedRegion {
    public:
    BoundedPlaneVector planes;

    cPlaneBoundedRegion() {}

    void addPlane(const cBoundedPlane3d &plane) {
        planes.push_back(plane);
    }

    void clear() {
        planes.clear();
    }

    unsigned int nPlanes() const {
        return planes.size();
    }

    void addToMesh(cMesh* mesh) const {
        for(BoundedPlaneVector::const_iterator it = planes.begin(); it != planes.end(); ++it) {
            it->addToMesh(mesh);
        }
    }

    // returns true if the point is outside of the region,
    // assumes the region is bounded by planes which do not intersect
    // then ray will point from the nearest point on the region boundary out to
    // that point
    inline bool rayToOutsidePoint(const cVector3d &pt, cRay3d &rayToPoint) const {
//        cRay3d *ptRays = new cRay3d[this->nPlanes()];
        cRay3d tempRayToPoint;
        double tempDistance, distance = DOUBLE_MAX;
        bool isOutside = false;

        // signed distances returned are positive if we're on the inside of the
        // object, and negative otherwise. Look for the largest negative
        // distance and keep that
        for(BoundedPlaneVector::const_iterator it = planes.begin(); it != planes.end(); ++it) {
            tempDistance = it->boundedRayToPoint(pt, tempRayToPoint);
            if(tempDistance < 0)
                isOutside = true;

            if(tempDistance <= distance) {
                // hold on to this one
                distance = tempDistance;
                rayToPoint.m_origin.copyfrom(tempRayToPoint.m_origin);
                rayToPoint.m_direction.copyfrom(tempRayToPoint.m_direction);
            }
        }

        return isOutside;
    }

    // get the bounded plane which is closest to a particular point
    // closestPlane will be set to a copy of that plane
    // returns the index of this plane in the planes vector
    unsigned int getIndexOfClosestBoundedPlane(const cVector3d &pt) const {
        cRay3d tempRayToPoint;
        double tempDistance, distance = DOUBLE_MAX;
        unsigned int indexCurrent, indexClosest;

        // signed distances returned are positive if we're on the inside of the
        // object, and negative otherwise. Look for the largest negative
        // distance and keep that
        indexCurrent = 0;
        for(BoundedPlaneVector::const_iterator it = planes.begin();
                it != planes.end(); ++it, ++indexCurrent) {
            tempDistance = fabs(it->boundedDistanceToPointSigned(pt));

            if(tempDistance <= distance) {
                // hold on to this one
                distance = tempDistance;
                indexClosest = indexCurrent;
            }
        }

        return indexClosest;
    }

    // given a plane, build a cPlaneBoundedRegion in dest using the provided centeredXYPlane as the X and Y axes,
    // adding a plane thickness/2 away along it's normal (top), -thickness/2 away along it's normal (bottom, in the other direction)
    // and adding the other four sides. Looking along the normal with "up" being the +Y axis of centeredXYplane, the other four planes are
    // up, down, left, and right
    // plane ordering in dest: bottom, top, up, down, left, right
    static void buildBoxFromBoundedPlane(const cBoundedPlane3d &bottom, double thickness, cPlaneBoundedRegion &dest) {
        cBoundedPlane3d top, left, right, up, down, temp;
        double halfX, halfY, halfZ;

        dest.clear();

        halfX = bottom.halfX;
        halfY = bottom.halfY;
        halfZ = thickness / 2.0;

        top.copyfrom(bottom);
        top.translateAlongNormal(thickness);
        top.invert();

        temp.copyfrom(bottom);
        temp.translateAlongNormal(halfZ);
        temp.getPerpendicularPlaneAlongVec2(left);
        left.halfX = halfZ;
        left.halfY = halfY;
        left.translateAlongNormal(-halfX);

        right.copyfrom(left);
        right.translateAlongNormal(2*halfX);
        right.invert();

        temp.getPerpendicularPlaneAlongVec1(down);
        down.halfX = halfX;
        down.halfY = halfZ;
        down.translateAlongNormal(-halfY);

        up.copyfrom(down);
        up.translateAlongNormal(2*halfY);
        up.invert();

        // add planes to dest, ordering is important for applications which reference planes by index
        dest.addPlane(bottom);
        dest.addPlane(top);
        dest.addPlane(up);
        dest.addPlane(down);
        dest.addPlane(left);
        dest.addPlane(right);
    }
};

// Implements a rotate, scale, and shift transformation
// can be applied (forward or inverse) to cVector3d, cBoundedPlane3d, and cPlaneBoundedRegion
// output = (mRotation * input) .* vGain + vOffset
struct CoordinateTransform {
    cVector3d vOffset;
    cVector3d vGain;
    cMatrix3d mRotation; // rotation of coordinate frame

    // cVector3d : transform with or without applying offset (points vs.
    // vectors)

    // apply forward transform to vector, including offset
    // for points
    void transform(const cVector3d &src, cVector3d &dest, bool offset=true) const
    {
        dest.copyfrom(src);
        mRotation.mul(dest);
        dest.mulElement(vGain);
        if(offset)
            dest.add(vOffset);
    }

    // apply forward transform, but only rotate and scale, ignore offset
    // for directional vectors
    void transformScale(const cVector3d &src, cVector3d &dest) const
    {
        this->transform(src, dest, false);
    }

    void inverse(const cVector3d &src, cVector3d &dest, bool offset=true) const
    {
        cVector3d vGainInv;
        cMatrix3d mRotationInv;

        vGainInv.copyfrom(vGain);
        vGainInv.x(1.0/vGainInv.x());
        vGainInv.y(1.0/vGainInv.y());
        vGainInv.z(1.0/vGainInv.z());

        mRotationInv.copyfrom(mRotation);
        mRotationInv.invert();

        dest.copyfrom(src);
        if(offset)
            dest.sub(vOffset);
        dest.mulElement(vGainInv);
        mRotationInv.mul(dest);
    }

    // apply inverse transform to vector, ignore offset
    // for directional vectors
    void inverseScale(const cVector3d &src, cVector3d &dest) const
    {
        this->inverse(src, dest, false);
    }

    // cBoundedPlane3d

    // apply forward transform to a plane
    void transform(const cBoundedPlane3d &src, cBoundedPlane3d &dest) const
    {
        dest.copyfrom(src);

        this->transformScale(src.vec1, dest.vec1);
        this->transformScale(src.vec2, dest.vec2);
        this->transform(src.thru, dest.thru);
        dest.update();

        // the half sizes along vec 1 and 2 must be scaled according to the rotation matrix and the gains
        cVector3d hvXs, hvXd, hvYs, hvYd;
        this->transformScale(hvXs, hvXd);
        this->transformScale(hvYs, hvYd);
        dest.halfX = hvXd.length();
        dest.halfY = hvYd.length();

    }

    void inverse(const cBoundedPlane3d &src, cBoundedPlane3d &dest) const
    {
        dest.copyfrom(src);

        this->inverseScale(src.vec1, dest.vec1);
        this->inverseScale(src.vec2, dest.vec2);
        this->inverse(src.thru, dest.thru);

        // the half sizes along vec 1 and 2 must be scaled according to the rotation matrix and the gains
        cVector3d hvXs, hvXd, hvYs, hvYd;
        src.halfVectors(hvXs, hvYs);
        this->inverseScale(hvXs, hvXd);
        this->inverseScale(hvYs, hvYd);
        dest.halfX = hvXd.length();
        dest.halfY = hvYd.length();

        dest.update();
    }

    // cPlaneBoundedRegion

    void transform(const cPlaneBoundedRegion &src, cPlaneBoundedRegion &dest) const
    {
        dest.clear();
        cBoundedPlane3d plane;
        for(BoundedPlaneVector::const_iterator it = src.planes.begin(); it != src.planes.end(); ++it) {
            this->transform(*it, plane);
            dest.addPlane(plane);
        }
    }

    void inverse(const cPlaneBoundedRegion &src, cPlaneBoundedRegion &dest) const
    {
        dest.clear();
        cBoundedPlane3d plane;
        for(BoundedPlaneVector::const_iterator it = src.planes.begin(); it != src.planes.end(); ++it) {
            this->inverse(*it, plane);
            dest.addPlane(plane);
        }
    }
};

#endif // ifndef CPlane3dH
