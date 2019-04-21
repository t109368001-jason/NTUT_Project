#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <math.h>
#include <vector>
#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glut.h>
#include <wingdi.h>
#include "include/viewer.h"
#include <thread>

#define VIEWER_WIDTH 1024.0f
#define VIEWER_HEIGHT 768.0f

pcl::PolygonMesh triangle;
std::stringstream fileName;

Viewer *viewer = new Viewer;
float task = 0.0;

void Mouse(int button, int state, int x, int y);
void mouseMove(int x, int y);
void ChangeSize(int w, int h);
void timer();
void Display(void);
void Keyboard(unsigned char key, int x, int y);
void SpecialKeys(int key, int x, int y);
void triangulation(pcl::PolygonMesh &triangle, std::string str);
void poission_surface(pcl::PolygonMesh &poission, std::string str);

void triangulation(pcl::PolygonMesh &triangle, std::string str)
{
	std::stringstream infile, outfile;

	infile << str <<".pcd";
	outfile << str << "_triangle.ply";
	if (pcl::io::loadPLYFile(outfile.str(), triangle) == -1)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile(&(*infile.str().begin()), cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);

		pcl::CentroidPoint<pcl::PointXYZ> centroid;
		pcl::PointXYZ cent;
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			centroid.add(cloud->points[i]);
		}
		centroid.get(cent);

		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			cloud->points[i].x -= cent.x;
			cloud->points[i].y -= cent.y;
			cloud->points[i].z -= cent.z;
		}

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(30);
		n.compute(*normals);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud(cloud_with_normals);

		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

		gp3.setSearchRadius(30);

		gp3.setMu(2.5);
		gp3.setMaximumNearestNeighbors(300);
		gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees
		gp3.setMinimumAngle(M_PI / 36); // 10 degrees
		gp3.setMaximumAngle(5 * M_PI / 6); // 120 degrees
		gp3.setNormalConsistency(false);

		gp3.setInputCloud(cloud_with_normals);
		gp3.setSearchMethod(tree2);
		gp3.reconstruct(triangle);
		pcl::io::savePLYFile(outfile.str(), triangle);
	}
}

void poission_surface(pcl::PolygonMesh &poission, std::string str)
{
	std::stringstream infile, outfile;
	infile << str << ".pcd";
	outfile << str << "_poission.ply";
	if (pcl::io::loadPLYFile(outfile.str(), poission) == -1)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile(&(*infile.str().begin()), cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);

		pcl::CentroidPoint<pcl::PointXYZ> centroid;
		pcl::PointXYZ cent;
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			centroid.add(cloud->points[i]);
		}
		centroid.get(cent);
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			cloud->points[i].x -= cent.x;
			cloud->points[i].y -= cent.y;
			cloud->points[i].z -= cent.z;
		}

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(30);
		n.compute(*normals);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud(cloud_with_normals);

		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth(8);
		poisson.setSolverDivide(8);
		poisson.setIsoDivide(8);
		poisson.setPointWeight(4.0f);
		poisson.setInputCloud(cloud_with_normals);

		poisson.reconstruct(poission);
		pcl::io::savePLYFile(outfile.str(), poission);
	}
}

void T_Reconstruct()
{
	int min = 50;
	int max = 100;
	int step = 5;
	float total = (float(max - min) / step) * 2;
	for (size_t i = min; i < max; i += step)
	{
		pcl::PolygonMesh triangle;
		std::stringstream infile;
		infile << "../file/";
		infile << i;
		//triangulation(triangle, infile.str());
		poission_surface(triangle, infile.str());
		viewer->Buffer.push(triangle);
		task += (1.0 / total);
		glutPostRedisplay();
	}
	for (size_t i = max; i > min; i -= step)
	{
		pcl::PolygonMesh triangle;
		std::stringstream infile;
		infile << "../file/";
		infile << i;
		//triangulation(triangle, infile.str());
		poission_surface(triangle, infile.str());
		viewer->Buffer.push(triangle);
		task += (1.0 / total);
		glutPostRedisplay();
	}
}

int main(int argc, char* argv[])
{
	HANDLE thread1;
	HANDLE thread2;
	DWORD tID1;
	DWORD tID2;
	thread1 = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)T_Reconstruct, 0, 0, &tID1);
	thread2 = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)timer, 0, 0, &tID1);
	
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(VIEWER_WIDTH, VIEWER_HEIGHT);
	glutCreateWindow("Point Clout Viewer");
	glutReshapeFunc(ChangeSize);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(Mouse);
	glutMotionFunc(mouseMove);
	glutSpecialFunc(SpecialKeys);
	glutDisplayFunc(Display);

	glutMainLoop();
	return 0;
}

void timer()
{
	while (1)
	{
		if (viewer->mode == Play)
		{
			//if ((clock() - viewer->count) > (1000 / viewer->FPS))
			{
				viewer->next = true;
				viewer->count = clock();
				glutPostRedisplay();
			}
		}
		Sleep(1000 / viewer->FPS);
	}
}

void Display(void)
{
	pcl::PolygonMesh temp;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	gluLookAt(viewer->location.getY(), viewer->location.getZ(), viewer->location.getX(), viewer->location.getY() + viewer->look.getY(), viewer->location.getZ() + viewer->look.getZ(), viewer->location.getX() + viewer->look.getX(), 0.0f, 1.0f, 0.0f);

	if (!viewer->Buffer.empty())
	{
		if (viewer->next == true)
		{
			viewer->next = false;
			temp = viewer->Buffer.front();
			viewer->Buffer.pop();
			viewer->draw(temp, viewer->fill);
			viewer->Buffer.push(temp);
		}
		else
		{
			viewer->draw(viewer->Buffer.front(), viewer->fill);
		}
		/*
		if (viewer->mode == Play)
		{
			if ((clock() - viewer->count) > (1000 / viewer->FPS))
			{
				temp = viewer->Buffer.front();
				viewer->Buffer.pop();
				viewer->draw(temp, viewer->fill);
				viewer->Buffer.push(temp);
				viewer->count = clock();
			}
			else
			{
				viewer->draw(viewer->Buffer.back(), viewer->fill);
			}
			glutPostRedisplay();
		}
		else if (viewer->mode == Pause)
		{
			viewer->draw(viewer->Buffer.front(), viewer->fill);
			glutPostRedisplay();
		*/
	}
	viewer->draw(10, 64, "W S A D : Move camera");
	viewer->draw(10, 48, "Up Down Left Right : Rotate camera");
	viewer->draw(10, 32, "Mouse drag : Rotate object");
	viewer->draw(10, 16, "Mouse scroll : Zoom");
	if (task < 1.0)
	{
		std::stringstream ss;
		ss << "Loading... (";
		ss << int(task * 100);
		ss << "%)";
		viewer->draw(800, 16, ss.str());
	}
	glutSwapBuffers();
}

void ChangeSize(int w, int h)
{
	if (h == 0)
		h = 1;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.001, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	glColor3f(1.0, 1.0, 1.0);
	glLineWidth(1.0);
}
void Mouse(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_LEFT_BUTTON:			//Mouse left buttton
		if (state == GLUT_UP) {
			viewer->xClick = -1;
			viewer->yClick = -1;
		}
		else {
			viewer->xClick = x;
			viewer->yClick = y;
		}
		break;
	case GLUT_MIDDLE_BUTTON:		//Mouse middle button

		break;
	case GLUT_RIGHT_BUTTON:			//Mouse right button

		break;
	case 3:		viewer->move(Forward);	break;
	case 4:		viewer->move(Backward);	break;
	default:	std::cout << "Mouse : " << button << "\tState : " << state << std::endl;	break;
	}
	glutPostRedisplay();
}
void mouseMove(int x, int y)
{
	viewer->moveAroud(x, y);
	glutPostRedisplay();
}
void Keyboard(unsigned char key, int x, int y)
{
	GLfloat fraction = CAMERA_MOVE_SPEED;
	switch (key)
	{
	case 'w':	viewer->move(Forward);	break;
	case 's':	viewer->move(Backward);	break;
	case 'a':	viewer->move(Left);		break;
	case 'd':	viewer->move(Right);	break;
	case 't':	viewer->screenshot();			break;
	case 'f':
		viewer->look.theta = M_PI - viewer->location.theta;
		viewer->look.phi = viewer->location.phi + M_PI;
		break;
	case 'l':	viewer->fill = !viewer->fill;	break;
	case 32:		// space
		if (viewer->mode == Play)
			viewer->mode = Pause;
		else
			if (task >= 1.0)
			{
				std::cout << "AAAAAAAAAAAAAAAAAAAAA" << std::endl;
				viewer->mode = Play;
			}
		break;
	default:	printf("   Keyboard %c == %d\n", key, key);	break;
	}
	glutPostRedisplay();
}
void SpecialKeys(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_LEFT:		viewer->rotation(0.0, 1.0);		break;
	case GLUT_KEY_RIGHT:	viewer->rotation(0.0, -1.0);	break;
	case GLUT_KEY_UP:		viewer->rotation(-1.0, 0.0);	break;
	case GLUT_KEY_DOWN:		viewer->rotation(1.0, 0.0);		break;
	}
	glutPostRedisplay();
}
