#ifndef PROJECTION_H
#define PROJECTION_H

//#include "../utils/rotation.h"
#include "../CBA/rotation.h"
// pose : 6 dims array with
// [0-2] : angle-axis rotation
// [3-5] : translation
// camera : 9 dims array with 
// [0-2] : fx, fy, s
// [3-4] : u, v
// [5-8] : k1, k2, t1, t2
// point : 3D location. 
// predictions : 2D predictions with center of the image plane. 
/*
template<typename T>
inline bool CamProjectionWithDistortion(const T *pose, const T *camera, const T *point, T *predictions) {
    // Rodrigues' formula
    T p[3];
    AngleAxisRotatePoint(pose, point, p);
    // pose[3,4,5] are the translation
    p[0] += pose[3];
    p[1] += pose[4];
    p[2] += pose[5];
    // Compute the center of distortion
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    // Apply second and fourth order radial distortion
    const T k1 = camera[5], k2 = camera[6], p1 = camera[7], p2 = camera[8];
    T r2 = xp * xp + yp * yp;
    T distortion = T(1.) + r2 * (k1 + k2 * r2);
    T xd = xp * distortion + T(2.) * p1 * xp * yp + p2 * (r2 + T(2.) * xp * xp);
    T yd = yp * distortion + T(2.) * p2 * xp * yp + p1 * (r2 + T(2.) * yp * yp);
    const T fx = camera[0], fy = camera[1], s = camera[2], u0 = camera[3], v0 = camera[4];
    predictions[0] = fx * xd + u0 + s * yd;
    predictions[1] = fy * yd + v0;
    return true;
}
*/

template<typename T>
inline bool CamProjectionWithDistortion(const T *pose, const T *point, T *predictions) {
	// Rodrigues' formula
	T p[3];
	AngleAxisRotatePoint(pose, point, p); //这个旋转和转移矩阵是从坐标系1到其他坐标系2/3/4的
	// pose[3,4,5] are the translation
	p[0] += pose[3];
	p[1] += pose[4];
	p[2] += pose[5];
	// Compute the center of distortion
	T xp = p[0] / p[2];
	T yp = p[1] / p[2];
	// Apply second and fourth order radial distortion ###############adding,check the order of the fy, pay attention
	T camera[9];
	FILE* fp = fopen("D:\\VSProject\\BA\\Config\\cameram.txt", "r");
	for (int i = 0; i<9; i++) {
		fscanf(fp, "%lf", &camera[i]);
	}
	fclose(fp);
	//在这里从txt里面读入camera参数
	const T k1 = camera[5], k2 = camera[6], p1 = camera[7], p2 = camera[8];
	T r2 = xp * xp + yp * yp;
	T distortion = T(1.) + r2 * (k1 + k2 * r2);
	T xd = xp * distortion + T(2.) * p1 * xp * yp + p2 * (r2 + T(2.) * xp * xp);
	T yd = yp * distortion + T(2.) * p2 * xp * yp + p1 * (r2 + T(2.) * yp * yp);
	const T fx = camera[0], fy = camera[1], s = camera[2], u0 = camera[3], v0 = camera[4];
	//predictions[0] = fx * xd + u0 + s * yd;
	predictions[0] = fx * xd + u0 ;
	predictions[1] = fy * yd + v0; //我就是怀疑在这里，它的系数太大了，导致其没收敛，偏得过于多了
	return true;
}

template<typename T>
inline bool
CamProjectionWithDistortion(const T *relative_pose, const T *pose, const T *camera, const T *point, T *predictions) {
    // Rodrigues' formula
    T p0[3];
    AngleAxisRotatePoint(pose, point, p0);
    // pose[3,4,5] are the translation
    p0[0] += pose[3];
    p0[1] += pose[4];
    p0[2] += pose[5];
    T p[3];
    AngleAxisRotatePoint(relative_pose, p0, p);
    p[0] += relative_pose[3];
    p[1] += relative_pose[4];
    p[2] += relative_pose[5];
    // Compute the center of distortion
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    // Apply second and fourth order radial distortion
    const T k1 = camera[5], k2 = camera[6], p1 = camera[7], p2 = camera[8];
    T r2 = xp * xp + yp * yp;
    T distortion = T(1.) + r2 * (k1 + k2 * r2);
    T xd = xp * distortion + T(2.) * p1 * xp * yp + p2 * (r2 + T(2.) * xp * xp);
    T yd = yp * distortion + T(2.) * p2 * xp * yp + p1 * (r2 + T(2.) * yp * yp);
    const T fx = camera[0], fy = camera[1], s = camera[2], u0 = camera[3], v0 = camera[4];
    predictions[0] = fx * xd + u0 + s * yd;
    predictions[1] = fy * yd + v0;
    return true;
}

#endif // projection.h