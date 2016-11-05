#include <iostream>
#include <math.h>
#include <GL/glut.h>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <windows.h>
#include "resource1.h"
#include <algorithm>
#include "MeshLib_core/Mesh.h"
#include "MeshLib_core/Iterators.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include "eig3.h"

using namespace std;

float whratio;
int win_height, win_width;
GLUquadricObj *obj;

/* Some variables to measure mouse movement	*/
int mousePositionX0 = 0, mousePositionY0 = 0;
int mouseButton = 0;

/* Some variables to describe the object dimension and camera placement	*/
float cameraPosition[3] = { 0, 0, 2 };	//default camera position
float objCenter[3] = { 0, 0, 0 };			//default object center
float boxMin[3] = { 0, 0, 0 };
float boxMax[3] = { 0, 0, 0 };
float axislen = 1.414;

/* Some variables to control interactive transformations	*/
float my_Rotation_x = 0, my_Rotation_y = 0;
float my_Translation[3] = { 0, 0, 0 };

bool using_face_normals_vertex_normals = false;
bool render_gaussian_curvature = false;
bool render_feature_lines = false;
bool render_principal_curvature = false;
bool render_extremalities = false;
bool render_surface = true;
bool render_edges = false;
bool render_collapsed;
bool tweakEx = false;
bool tweakDi = false;
bool tweakPrinc = false;
double curvatureThreshold[2] = { -0.05, 0.05 };
double extremality_upper_threshold = 1;
double eDefault = 1;
double principal_curvature_lower_threshold = 1e-20;   //this value must be reset for each mesh
double pDefault = 1e-20;
double dihedral_angle_upper_threshold = M_PI * 8.0 / 9.0;
double aDefault = M_PI * 8.0 / 9.0;
double PrinExpon = -15.0;

/* Some variables to store the triangular mesh and associated scalar fields*/
Mesh * pmesh;
Mesh * smooth_feature_line_mesh;
Point * face_normals;
Point * vertex_normals;
double * heTrgAngle;
double * gaussian_curvatures;

/* Variables for feature line detection */
double * mean_curvatures;       //edge-based
Point * edge_normals;           //edge-based, obviously
double * dihedral_angles;       //edge-based
double * k_max;                 //vertex-based
Point * k_max_vector;           //vertex-based
double * k_min;                 //vertex-based
Point * k_min_vector;           //vertex-based
double * e_min;                 //vertex-based
double * e_max;                  //vertex-based
int number_of_positive_feature_line_vertices;
int number_of_negative_feature_line_vertices;
bool * feature_line_edges;
bool * collapsed_feature_line_edges;
bool * positive_feature_line_vertices;  //vertex-based
bool * negative_feature_line_vertices;  //vertex-based
bool * positive_feature_line_faces;     //face-based
bool * negative_feature_line_faces;     //face-based
bool * regular_triangles;               //face-based
bool * semi_regular_triangles;          //face-based
struct v_2_center
{
	Vertex *v;
	Point c;
};

struct vertex_pair
{
	double v0[3];
	double v1[3];
};

vector<v_2_center> addOns;
vector<Point> intra_nodal_lines;

//stored as k_min_vector_signs in [0:2] and k_max_vector_signs in [3:5]
//true if sign change is needed (= true if negative)
bool ** principal_curvature_signs;      //face-based, then vertex-based

/* Some variables to store the topological invariants*/
int N_c, N_b, N_g;

void MyInit(void);

void ComputeBoundingBox();
void ComputeNormal();
void ComputeGaussianCurvature();
void ComputeConnectedComponentNumber();
void ComputeBoundaryNumber();
void ComputeGenusNumber();

void detect_feature_lines();
void process_singular_triangles();
void findEdges();
void collapseFaces();
void writeToConsole();
void writeMesh();

//rendering
void SetCamera(void);
void Render_BoxAndAxes(void);
void Render_Mesh();

//glut functions
void mouseMove(int x, int y);
void mouseClick(int button, int state, int x, int y);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);

ostream& operator<<(ostream& output_stream, const Point& vertex)
{
	output_stream << "( " << vertex.v[0] << " , " << vertex.v[1] << " , " << vertex.v[2] << " )";
	return output_stream;
}

void keyUp(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'b':
		tweakEx = false;
		detect_feature_lines();
		process_singular_triangles();
		findEdges();
		collapseFaces();
		cout << string(100, '\n');
		writeToConsole();
		glutPostRedisplay();
		break;
	case 'n':
		tweakDi = false;
		detect_feature_lines();
		process_singular_triangles();
		findEdges();
		collapseFaces();
		cout << string(100, '\n');
		writeToConsole();
		glutPostRedisplay();
		break;
	case 'm':
		tweakPrinc = false;
		detect_feature_lines();
		process_singular_triangles();
		findEdges();
		collapseFaces();
		cout << string(100, '\n');
		writeToConsole();
		glutPostRedisplay();
		break;
	case 'd':
		cout << string(100, '\n');
		writeToConsole();
		glutPostRedisplay();
		break;
	case'f':
		writeMesh();
		cout << "Mesh Files Written." << endl;
	default:
		break;
	}
}

void writeToConsole(void)
{
	cout << "Use different keys to visualize different values." << endl
		<< "'k' to display Gaussian curvature" << endl
		<< "'p' to display principal curvature" << endl
		<< "'e' to display extremalities" << endl
		<< "'l' to display coloring by feature detection" << endl
		<< "'s' to toggle vertex/face normals" << endl
		<< "'d' to set all values to default" << endl
		<< "'w' to toggle wireframe feature lines" << endl
		<< "'c' to toggle wireframe feature lines after faceCollapse function" << endl
		<< "'f' to write 'featureMesh.mesh' and 'featureMeshFaceCollapse.mesh'" << endl
		<< "These files contain only Vertices in order. Render with 'GL_LINES'" << endl
		<< "Toggle on both 'w' and 'c' to see a representation of how faceCollapse operates" << endl << endl
		<< "The following commands require any mouse button held down" << endl
		<< "Hold 'b' and move mouse up/down to shift Extremality_Upper_Threshold" << endl
		<< "Hold 'n' and move mouse up/down to shift Dihedral_Angle_Upper_Threshold" << endl
		<< "Hold 'm' and move mouse up/down to shift Principal_Curvature_Lower_Threshold" << endl
		<< "-----------------------------------------------------------------------------------------" << endl
		<< "Extremality_Upper_Threshold: " << extremality_upper_threshold << endl
		<< "Dihedral_Angle_Upper_Threshold: " << dihedral_angle_upper_threshold << endl
		<< "Principal_Curvature_Lower_Threshold: " << principal_curvature_lower_threshold << endl;
}

void collapseFaces(void)
{

	int count = 0;
	bool *bulkFaces;
	bool *visitedFaces;

	bool *vertices2;
	bool addToEntries = true;
	bulkFaces = new bool[pmesh->numFaces()];
	visitedFaces = new bool[pmesh->numFaces()];
	collapsed_feature_line_edges = new bool[pmesh->numEdges()];

	vertices2 = new bool[pmesh->numVertices()];
	vector<Face*> group;
	vector<Face*> toProcess;
	vector<Vertex*> groupVertices;
	vector<Vertex*> entries;
	vector<Edge*> mark_for_delete;
	Point center;

	for (int j = 0; j < pmesh->numFaces(); j++)
	{
		bulkFaces[j] = false;
		visitedFaces[j] = false;
	}

	for (int j = 0; j < pmesh->numVertices(); j++)
	{
		
		vertices2[j] = false;
	}

	for (MeshFaceIterator fit(pmesh); !fit.end(); ++fit)
	{
		Face *f = *fit;
		FaceEdgeIterator eit(f);
		Edge *e1 = *eit;
		++eit;
		Edge *e2 = *eit;
		++eit;
		Edge *e3 = *eit;

		if (feature_line_edges[e1->index()] && feature_line_edges[e2->index()] && feature_line_edges[e3->index()])
		{
			bulkFaces[f->index()] = true;
		}
	}

	for (int j = 0; j < pmesh->numFaces(); j++)
	{
		if (bulkFaces[j] && !visitedFaces[j])
		{
			visitedFaces[j] = true;
			group.push_back(pmesh->indFace(j));
			for (FaceHalfedgeIterator hit(pmesh->indFace(j)); !hit.end(); ++hit)
			{
				Halfedge *he = *hit;

				if (he->twin() && bulkFaces[he->twin()->face()->index()])
				{
					group.push_back(he->twin()->face());
					toProcess.push_back(he->twin()->face());
					visitedFaces[he->twin()->face()->index()] = true;
					
				}
			}

			while (!toProcess.empty())
			{
				Face *current = toProcess[toProcess.size() - 1];
				toProcess.pop_back();

				for (FaceHalfedgeIterator hit(current); !hit.end(); ++hit)
				{
					Halfedge *he = *hit;
					if (he->twin() && bulkFaces[he->twin()->face()->index()] && !visitedFaces[he->twin()->face()->index()])
					{
						group.push_back(he->twin()->face());
						toProcess.push_back(he->twin()->face());
						visitedFaces[he->twin()->face()->index()] = true;
					}
				}
			}
			
			for (unsigned int j = 0; j < group.size(); j++)
			{
				Face *f = group[j];
				for (FaceVertexIterator vit(f); !vit.end(); ++vit)
				{
					Vertex *v = *vit;
					bool doAdd = true;

					for (unsigned int i = 0; i < groupVertices.size(); i++)
					{
						if (groupVertices[i] == v)
						{
							doAdd = false;
						}
					}

					if (doAdd)
					{
						groupVertices.push_back(v);
					}
				}
			}

			center = Point(0, 0, 0);

			
			for (unsigned int j = 0; j < groupVertices.size(); j++)
			{
				center += groupVertices[j]->point();
				addToEntries = false;

				for (VertexVertexIterator vit(groupVertices[j]); !vit.end(); ++vit)
				{
					Vertex *v = *vit;
					Edge *e = pmesh->vertexEdge(groupVertices[j], v);

					bool not_in_group = true;

					for (unsigned int m = 0; m < groupVertices.size(); m++)
					{
						if (groupVertices[m] == v)
						{
							not_in_group = false;
						}
					}

					if (feature_line_edges[e->index()] && not_in_group == true)
					{
						addToEntries = true;
					}

				}

				if (addToEntries)
				{
					
					entries.push_back(groupVertices[j]);
				}
			}

			center /= groupVertices.size();
			//printf("%f, %f, %f\n", center[0], center[1], center[2]);
			
			for (unsigned int k = 0; k < entries.size(); k++)
			{
				addOns.push_back(v_2_center());
				addOns.back().c = center;
				addOns.back().v = entries[k];
			}

			for (unsigned int l = 0; l < group.size(); l++)
			{
				Face *f = group[l];

				for (FaceEdgeIterator eit(f); !eit.end(); ++eit)
				{
					Edge *e = *eit;
					mark_for_delete.push_back(e);
				}
			}
			

			
		}

		
		toProcess.clear();
		group.clear();
		entries.clear();
		groupVertices.clear();
	}

	for (MeshEdgeIterator eit(pmesh); !eit.end(); ++eit)
	{
		Edge *e = *eit;
		collapsed_feature_line_edges[e->index()] = feature_line_edges[e->index()];
	}

	for (unsigned int j = 0; j < mark_for_delete.size(); j++)
	{
		collapsed_feature_line_edges[mark_for_delete[j]->index()] = false;
	}

	mark_for_delete.clear();
	delete[] bulkFaces;
	delete[] visitedFaces;
	delete[] vertices2;
}
void findEdges(void)
{
	feature_line_edges = new bool[pmesh->numEdges()];

	for (int j = 0; j < pmesh->numEdges(); j++)
	{
		feature_line_edges[j] = false;
	}

	for (MeshEdgeIterator eit(pmesh); !eit.end(); ++eit)
	{
		Edge *e = *eit;
		Halfedge *he = e->he(0);
		Vertex *v1 = he->source();
		Vertex *v2 = he->target();

		if (positive_feature_line_vertices[v1->index()] || negative_feature_line_vertices[v1->index()])
		{
			if (positive_feature_line_vertices[v2->index()] || negative_feature_line_vertices[v2->index()])
			{
				feature_line_edges[e->index()] = true;
			}
		}

	}
}

void writeMesh(void)
{
	int newIndex = 1;
	std::ofstream out1, out2;

	out1.open("featureMesh.mesh");
	out2.open("featureMeshFaceCollapse.mesh");

	for (MeshEdgeIterator eit(pmesh); !eit.end(); ++eit)
	{
		Edge *e = *eit;

		if (feature_line_edges[e->index()])
		{
			Halfedge *he = e->he(0);
			Vertex *v1 = he->source();
			Vertex *v2 = he->target();
			Point p1 = v1->point();
			Point p2 = v2->point();

			if (newIndex != 1)
			{
				out1 << std::endl;
			}

			out1 << "Vertex " << " " << newIndex << " " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;

			newIndex++;

			out1 << "Vertex " << " " << newIndex << " " << p2[0] << " " << p2[1] << " " << p2[2];

			newIndex++;
		}

	}

	newIndex = 1;

	for (MeshEdgeIterator eit(pmesh); !eit.end(); ++eit)
	{
		Edge *e = *eit;

		if (collapsed_feature_line_edges[e->index()])
		{
			Halfedge *he = e->he(0);
			Vertex *v1 = he->source();
			Vertex *v2 = he->target();
			Point p1 = v1->point();
			Point p2 = v2->point();

			if (newIndex != 1)
			{
				out2 << std::endl;
			}

			out2 << "Vertex " << " " << newIndex << " " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;

			newIndex++;

			out2 << "Vertex " << " " << newIndex << " " << p2[0] << " " << p2[1] << " " << p2[2];

			newIndex++;
		}

	}

	for (unsigned int j = 0; j < addOns.size(); j++)
	{
		Point v1 = addOns[j].v->point();
		Point v2 = addOns[j].c;

		if (newIndex != 1)
		{
			out2 << std::endl;
		}

		out2 << "Vertex " << " " << newIndex << " " << v1[0] << " " << v1[1] << " " << v1[2] << std::endl;

		newIndex++;

		out2 << "Vertex " << " " << newIndex << " " << v2[0] << " " << v2[1] << " " << v2[2];

		newIndex++;
	}

	

	out1.close();
	out2.close();
}

double constrain(double number, double min, double max)
{
	double result;

	if (number <= min)
		result = min;
	else if (number >= max)
		result = max;
	else
		result = number;

	return result;
}

int sign(double number)
{
	int result;

	if (number > 0)
		result = 1;
	else if (number < 0)
		result = -1;
	else //if(number == 0)
		result = 0;

	return result;
}

void process_singular_triangles(void)
{
	for (MeshFaceIterator face_iterator(pmesh); !face_iterator.end(); ++face_iterator)
	{
		Face * face = *face_iterator;

		double regular_neighbors = 0;

		for (FaceEdgeIterator edge_iterator(face); !edge_iterator.end(); ++edge_iterator)
		{
			Edge * edge = *edge_iterator;

			if (!edge->boundary())
			{
				//choose the neighboring face (just distinguish it from the current face)
				Face * neighbor = edge->he(0)->face()->index() == face->index() ? edge->he(1)->face() : edge->he(0)->face();

				if (face->index() == neighbor->index())
					cout << "error choosing the neighboring face" << endl;

				if (regular_triangles[neighbor->index()] || semi_regular_triangles[neighbor->index()])
					regular_neighbors++;
			}
		}

		if (regular_neighbors > 1)
			semi_regular_triangles[face->index()] = true;
		else
			semi_regular_triangles[face->index()] = false;
	}
}

void detect_feature_lines(void)
{

	//initialize
	for (MeshVertexIterator vertex_iterator(pmesh); !vertex_iterator.end(); ++vertex_iterator)
	{
		Vertex * vertex = *vertex_iterator;
		positive_feature_line_vertices[vertex->index()] = false;
		negative_feature_line_vertices[vertex->index()] = false;
	}

	for (MeshFaceIterator face_iterator(pmesh); !face_iterator.end(); ++face_iterator)
	{
		Face * face = *face_iterator;

		//initialize
		positive_feature_line_faces[face->index()] = false;
		negative_feature_line_faces[face->index()] = false;

		if (regular_triangles[face->index()])
		{
			Point e_min_gradient, e_min_gradient_term_1 = Point(0, 0, 0), e_min_gradient_term_2 = Point(0, 0, 0);
			Point e_max_gradient, e_max_gradient_term_1 = Point(0, 0, 0), e_max_gradient_term_2 = Point(0, 0, 0);

			//get all three vertices of the current triangle
			Vertex * ui, *uj, *uk;
			FaceVertexIterator vertex_iterator_2(face);
			ui = *vertex_iterator_2;
			++vertex_iterator_2;
			uj = *vertex_iterator_2;
			++vertex_iterator_2;
			uk = *vertex_iterator_2;

			// ∇f(u) = (fj - fi) * (ui - uk)perp / (2 * area) + (fk - fi) * (uj - ui)perp / (2 * area)

			//first get counterclockwise perpendicular vectors (face_normal ^ edge_vector)
			Point ui_uk_perp = ui->point() - uk->point();
			ui_uk_perp = face_normals[face->index()] ^ ui_uk_perp;

			Point uj_ui_perp = uj->point() - ui->point();
			uj_ui_perp = face_normals[face->index()] ^ uj_ui_perp;

			//next get area
			Point ui_uk = ui->point() - uk->point();
			Point ui_uj = ui->point() - uj->point();
			double area_of_triangle = abs((ui_uk ^ ui_uj).norm()) / 2.0;

			//finally, calculate gradients from the formula :
			// ∇f(u) = (fj - fi) * (ui - uk)perp / (2 * area) + (fk - fi) * (uj - ui)perp / (2 * area)
			e_min_gradient_term_1 = ui_uk_perp * (e_min[uj->index()] - e_min[ui->index()]) / (2 * area_of_triangle);
			e_min_gradient_term_2 = uj_ui_perp * (e_min[uk->index()] - e_min[ui->index()]) / (2 * area_of_triangle);
			e_min_gradient = e_min_gradient_term_1 + e_min_gradient_term_2;

			e_max_gradient_term_1 = ui_uk_perp * (e_max[uj->index()] - e_min[ui->index()]) / (2 * area_of_triangle);
			e_max_gradient_term_2 = uj_ui_perp * (e_max[uk->index()] - e_max[ui->index()]) / (2 * area_of_triangle);
			e_max_gradient = e_max_gradient_term_1 + e_max_gradient_term_2;

			//get consistent signs for the principal curvatures and their vectors
			//change signs for the principal curvature vectors and scalars of all three vertices
			double k_min_ui = k_min[ui->index()] * pow(-1, principal_curvature_signs[face->index()][0]);
			double k_min_uj = k_min[uj->index()] * pow(-1, principal_curvature_signs[face->index()][1]);
			double k_min_uk = k_min[uk->index()] * pow(-1, principal_curvature_signs[face->index()][2]);

			double k_max_ui = k_max[ui->index()] * pow(-1, principal_curvature_signs[face->index()][3]);
			double k_max_uj = k_max[uj->index()] * pow(-1, principal_curvature_signs[face->index()][4]);
			double k_max_uk = k_max[uk->index()] * pow(-1, principal_curvature_signs[face->index()][5]);

			Point k_min_vector_ui = k_min_vector[ui->index()] * principal_curvature_signs[face->index()][0];
			Point k_min_vector_uj = k_min_vector[uj->index()] * principal_curvature_signs[face->index()][1];
			Point k_min_vector_uk = k_min_vector[uk->index()] * principal_curvature_signs[face->index()][2];

			Point k_max_vector_ui = k_max_vector[ui->index()] * principal_curvature_signs[face->index()][3];
			Point k_max_vector_uj = k_max_vector[uj->index()] * principal_curvature_signs[face->index()][4];
			Point k_max_vector_uk = k_max_vector[uk->index()] * principal_curvature_signs[face->index()][5];

			double k_min_sum = k_min_ui + k_min_uj + k_min_uk;
			double k_max_sum = k_max_ui + k_max_uj + k_max_uk;

			Point k_min_vector_sum = k_min_vector_ui + k_min_vector_uj + k_min_vector_uk;
			Point k_max_vector_sum = k_max_vector_ui + k_max_vector_uj + k_max_vector_uk;

			//detect possible positive feature lines
			// <∇e_max, sum(k_max(T))> < 0 and abs( sum(k_max(T)) ) > abs( sum(k_min(T)) )

			//changed these to global variables
			
			

			//            //testing
			//            extremality_upper_threshold = 0.05;
			//            principal_curvature_lower_threshold = 1e-5;

			//            //star
			//            extremality_upper_threshold = 1;
			//            principal_curvature_lower_threshold = 1e-2;

			//            //hand
			//            extremality_upper_threshold = 0.05;
			//            principal_curvature_lower_threshold = 1e-5;

			//detect positive feature lines
			if (e_max_gradient * k_max_vector_sum < 0 &&        //condition 5
				abs(k_max_sum) > abs(k_min_sum))                             //condition 6
			{
				for (FaceVertexIterator vertex_iterator(face); !vertex_iterator.end(); ++vertex_iterator)
				{
					Vertex * vertex = *vertex_iterator;

					positive_feature_line_vertices[vertex->index()] = abs(e_max[vertex->index()]) < extremality_upper_threshold && abs(k_max[vertex->index()]) > principal_curvature_lower_threshold; //e_max 0 set non-empty?
					bool noise = true;

					for (VertexEdgeIterator veit(vertex); !veit.end(); ++veit)
					{
						Edge *e = *veit;
						if (dihedral_angles[e->index()] < dihedral_angle_upper_threshold)
						{
							noise = false;
						}
					}
					positive_feature_line_vertices[vertex->index()] = noise ? false : positive_feature_line_vertices[vertex->index()];
				}
				positive_feature_line_faces[face->index()] = positive_feature_line_vertices[ui->index()]
					|| positive_feature_line_vertices[uj->index()]
					|| positive_feature_line_vertices[uk->index()];
			}

			//detect negative feature lines
			if (e_min_gradient * k_min_vector_sum > 0 &&        //analogous to condition 5
				abs(k_max_sum) < abs(k_min_sum))                //analogous to condition 6
			{
				for (FaceVertexIterator vertex_iterator(face); !vertex_iterator.end(); ++vertex_iterator)
				{
					Vertex * vertex = *vertex_iterator;

					negative_feature_line_vertices[vertex->index()] = abs(e_min[vertex->index()]) < extremality_upper_threshold && abs(k_min[vertex->index()]) > principal_curvature_lower_threshold;//e_min 0 set non-empty?
					bool noise = true;

					for (VertexEdgeIterator veit(vertex); !veit.end(); ++veit)
					{
						Edge *e = *veit;
						if (dihedral_angles[e->index()] < dihedral_angle_upper_threshold)
						{
							noise = false;
						}
					}
					negative_feature_line_vertices[vertex->index()] = noise ? false : negative_feature_line_vertices[vertex->index()];
				}
				negative_feature_line_faces[face->index()] = negative_feature_line_vertices[ui->index()]
					|| negative_feature_line_vertices[uj->index()]
					|| negative_feature_line_vertices[uk->index()];
			}
		}
	}
}

//we calculate e_min and e_max at the same time because they are analogous equations
void calculate_extremalities(void)
{
	for (MeshVertexIterator vertex_iterator(pmesh); !vertex_iterator.end(); ++vertex_iterator)
	{
		double area_of_star = 0;

		Vertex * vertex = *vertex_iterator;

		//initialize extremality values before summing
		e_min[vertex->index()] = 0;
		e_max[vertex->index()] = 0;

		for (VertexFaceIterator face_iterator(vertex); !face_iterator.end(); ++face_iterator)
		{
			Face * face = *face_iterator;

			Point k_min_gradient, k_min_gradient_term_1 = Point(0, 0, 0), k_min_gradient_term_2 = Point(0, 0, 0);
			Point k_max_gradient, k_max_gradient_term_1 = Point(0, 0, 0), k_max_gradient_term_2 = Point(0, 0, 0);

			//get all three vertices of the current triangle
			Vertex * ui, *uj, *uk;
			FaceVertexIterator vertex_iterator_2(face);
			ui = *vertex_iterator_2;
			++vertex_iterator_2;
			uj = *vertex_iterator_2;
			++vertex_iterator_2;
			uk = *vertex_iterator_2;

			//identify the original vertex in terms of ui, uj, and uk to be able to use its consistent sign
			//the sign array is stored as {k_min_sign( ui ), k_min_sign( uj ), k_min_sign( uk ), k_max_sign( ui ), k_max_sign( uj ), k_max_sign( uk )}
			int k_min_sign_index;
			int k_max_sign_index;
			if (vertex == ui)
			{
				k_min_sign_index = 0;
				k_max_sign_index = 3;
			}
			else if (vertex == uj)
			{
				k_min_sign_index = 1;
				k_max_sign_index = 4;
			}
			else if (vertex == uk)
			{
				k_min_sign_index = 2;
				k_max_sign_index = 5;
			}
			else
			{
				cout << "error in picking 'vertex' from {ui, uj, uk}" << endl;
			}

			// ∇f(u) = (fj - fi) * (ui - uk)perp / (2 * area) + (fk - fi) * (uj - ui)perp / (2 * area)

			//first get counterclockwise perpendicular vectors (face_normal ^ edge_vector)
			Point ui_uk_perp = ui->point() - uk->point();
			ui_uk_perp = face_normals[face->index()] ^ ui_uk_perp;

			Point uj_ui_perp = uj->point() - ui->point();
			uj_ui_perp = face_normals[face->index()] ^ uj_ui_perp;

			//next get area
			Point ui_uk = ui->point() - uk->point();
			Point ui_uj = ui->point() - uj->point();
			double area_of_triangle = abs((ui_uk ^ ui_uj).norm()) / 2.0;

			//check indices of principal curvature signs
			//change signs for the principal curvature vectors and scalars of all three vertices
			double k_min_ui = k_min[ui->index()] * pow(-1, principal_curvature_signs[face->index()][0]);
			double k_min_uj = k_min[uj->index()] * pow(-1, principal_curvature_signs[face->index()][1]);
			double k_min_uk = k_min[uk->index()] * pow(-1, principal_curvature_signs[face->index()][2]);

			double k_max_ui = k_max[ui->index()] * pow(-1, principal_curvature_signs[face->index()][3]);
			double k_max_uj = k_max[uj->index()] * pow(-1, principal_curvature_signs[face->index()][4]);
			double k_max_uk = k_max[uk->index()] * pow(-1, principal_curvature_signs[face->index()][5]);

			Point k_min_vector_ui = k_min_vector[ui->index()] * principal_curvature_signs[face->index()][0];
			Point k_min_vector_uj = k_min_vector[uj->index()] * principal_curvature_signs[face->index()][1];
			Point k_min_vector_uk = k_min_vector[uk->index()] * principal_curvature_signs[face->index()][2];

			Point k_max_vector_ui = k_max_vector[ui->index()] * principal_curvature_signs[face->index()][3];
			Point k_max_vector_uj = k_max_vector[uj->index()] * principal_curvature_signs[face->index()][4];
			Point k_max_vector_uk = k_max_vector[uk->index()] * principal_curvature_signs[face->index()][5];

			//finally, calculate gradients from the formula :
			// ∇f(u) = (fj - fi) * (ui - uk)perp / (2 * area) + (fk - fi) * (uj - ui)perp / (2 * area)
			k_min_gradient_term_1 = ui_uk_perp * (k_min_uj - k_min_ui) / (2 * area_of_triangle);
			k_min_gradient_term_2 = uj_ui_perp * (k_min_uk - k_min_ui) / (2 * area_of_triangle);
			k_min_gradient = k_min_gradient_term_1 + k_min_gradient_term_2;

			k_max_gradient_term_1 = ui_uk_perp * (k_max_uj - k_max_ui) / (2 * area_of_triangle);
			k_max_gradient_term_2 = uj_ui_perp * (k_max_uk - k_max_ui) / (2 * area_of_triangle);
			k_max_gradient = k_max_gradient_term_1 + k_max_gradient_term_2;

			//sum this for the final step which is the outer division
			area_of_star += area_of_triangle;

			//do the actual summation for both e_min and e_max
			Point consistent_k_min = k_min_vector[vertex->index()] * pow(-1, principal_curvature_signs[face->index()][k_min_sign_index]);
			Point consistent_k_max = k_max_vector[vertex->index()] * pow(-1, principal_curvature_signs[face->index()][k_max_sign_index]);
			e_min[vertex->index()] += (k_min_gradient * consistent_k_min) * area_of_triangle;
			e_max[vertex->index()] += (k_max_gradient * consistent_k_max) * area_of_triangle;
		}

		//simply divide by the area of the star of triangles surrounding the current vertex
		e_min[vertex->index()] /= area_of_star;
		e_max[vertex->index()] /= area_of_star;

	}

}

void set_principal_curvature_threshold(void)
{
	double *k_max_copy;
	k_max_copy = new double[pmesh->numVertices()];

	for (int i = 0; i < pmesh->numVertices(); i++)
	{
		k_max_copy[i] = k_max[i];
	}

	sort(k_max_copy, k_max_copy + pmesh->numVertices());

	//    for(int i = 0; i < pmesh->numVertices(); i ++)
	//    {
	//        cout << k_max_copy[i] << endl;
	//    }

	double max = k_max_copy[pmesh->numVertices() - 1];
	//cout << "MAX = " << max << endl;
	double min = k_max_copy[0];

	const int number_of_intervals = 20;
	int occurence_frequency[number_of_intervals];
	int index = 0;
	for (int interval = 0; interval < number_of_intervals; interval++)
	{
		//initialize
		occurence_frequency[interval] = 0;

		//find number of vertices with k_max within the range for the specified interval
		double interval_max = (min + ((max - min) / (number_of_intervals - interval)));
		while (k_max_copy[index] < interval_max)
		{
			occurence_frequency[interval] ++;
			index++;
		}
		//cout << "number_of_intervals - interval: " << number_of_intervals - interval << endl;
		//cout << occurence_frequency[interval] << " curvatures < " << interval_max << endl;
	}

	int interval = 0;
	int total_occurences = 0;
	do
	{
		double interval_max = (min + ((max - min) / (number_of_intervals - interval)));
		total_occurences += occurence_frequency[interval];
		principal_curvature_lower_threshold = interval_max;
		interval++;
	} while (total_occurences < pmesh->numVertices() * 0.85);

	//cout << "min k_max: " << min << endl;
	//cout << "max k_max: " << max << endl;
	//cout << "max - min: " << max - min << endl;
	//cout << "principal_curvature_lower_threshold: " << principal_curvature_lower_threshold << endl;

	delete[] k_max_copy;

	pDefault = principal_curvature_lower_threshold;
}


void detect_regular_triangles_and_choose_principal_curvature_signs(void)
{
	for (MeshFaceIterator face_iterator(pmesh); !face_iterator.end(); ++face_iterator)
	{
		Face * face = *face_iterator;

		//initialize
		regular_triangles[face->index()] = false;

		//get all vertices of the triangle
		Vertex * ui, *uj, *uk;

		FaceVertexIterator vertex_iterator(face);
		ui = *vertex_iterator;
		++vertex_iterator;
		uj = *vertex_iterator;
		++vertex_iterator;
		uk = *vertex_iterator;

		Point k_min_vector_local[3];
		Point k_max_vector_local[3];

		bool k_min_consistent = false;
		bool k_max_consistent = false;

		//regular_triangles[face->index()] = true;

		double threshold = 0;

		//choose k_min_vectors
		for (int i0 = 0; i0 < 2; i0++)
		{
			for (int i1 = 0; i1 < 2; i1++)
			{
				for (int i2 = 0; i2 < 2; i2++)
				{
					//set signs for Kmin
					k_min_vector_local[0] = k_min_vector[ui->index()] * pow(-1, i0);
					k_min_vector_local[1] = k_min_vector[uj->index()] * pow(-1, i1);
					k_min_vector_local[2] = k_min_vector[uk->index()] * pow(-1, i2);

					//check if Ki(p)*Ki(q) all are positive
					if ((k_min_vector_local[0] * k_min_vector_local[1] >= -threshold)
						&& (k_min_vector_local[1] * k_min_vector_local[2] >= -threshold)
						&& (k_min_vector_local[2] * k_min_vector_local[0] >= -threshold))
					{
						//regular_triangles[face->index()] = true;
						//signs for k_min_vector can be chosen consistently on this face
						k_min_consistent = true;

						//and store the consistently-chosen principal curvatures
						principal_curvature_signs[face->index()][0] = pow(-1, i0) == -1 ? 1 : 0;
						principal_curvature_signs[face->index()][1] = pow(-1, i1) == -1 ? 1 : 0;
						principal_curvature_signs[face->index()][2] = pow(-1, i2) == -1 ? 1 : 0;
					}
				}
			}
		}

		//choose k_max_vectors
		for (int i0 = 0; i0 < 2; i0++)
		{
			for (int i1 = 0; i1 < 2; i1++)
			{
				for (int i2 = 0; i2 < 2; i2++)
				{

					//regular_triangles[face->index()] = true;

					//set signs for ->Kmax
					k_max_vector_local[0] = k_max_vector[ui->index()] * pow(-1, i0);
					k_max_vector_local[1] = k_max_vector[uj->index()] * pow(-1, i1);
					k_max_vector_local[2] = k_max_vector[uk->index()] * pow(-1, i2);

					//check if Ki(p)*Ki(q) all are positive
					if ((k_max_vector_local[0] * k_max_vector_local[1] >= -threshold)
						&& (k_max_vector_local[1] * k_max_vector_local[2] >= -threshold)
						&& (k_max_vector_local[2] * k_max_vector_local[0] >= -threshold))
					{

						//regular_triangles[face->index()] = true;
						//signs for k_max_vector can be chosen consistently on this face
						k_max_consistent = true;

						//and store the consistently-chosen principal curvatures
						principal_curvature_signs[face->index()][3] = pow(-1, i0) == -1 ? 1 : 0;
						principal_curvature_signs[face->index()][4] = pow(-1, i1) == -1 ? 1 : 0;
						principal_curvature_signs[face->index()][5] = pow(-1, i2) == -1 ? 1 : 0;

						//                        //test to see if subsequent triangles change the values in k_max_vector
						//                        k_max_vector[ui->index()] *= pow(-1, i0);
						//                        k_max_vector[uj->index()] *= pow(-1, i1);
						//                        k_max_vector[uk->index()] *= pow(-1, i2);

					}
				}
			}
		}

		//simply mark the triangle as regular if both k_min_vector and k_max_vector can be chosen consistently
		if (k_min_consistent && k_max_consistent)
		{
			regular_triangles[face->index()] = true;
		}

		//test to see if subsequent triangles change the values in k_max_vector
		//cout << k_max_vector[15] << endl;

	}
}

/*
* step 1 (inner loop) : calculate the edge-based shape operators for a single vertex
* step 2 (outer loop) : calculate the vertex-based shape operator
* step 3 (outer loop, after inner loop) : find eigenvalues and eigenvectors of the vertex-based shape operator and store in global variables
*/
void calculate_principal_curvatures(void)
{

	for (MeshVertexIterator vertex_iterator(pmesh); !vertex_iterator.end(); ++vertex_iterator)
	{
		Vertex * vertex = *vertex_iterator;

		double vertex_based_shape_operator[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

		for (VertexEdgeIterator edge_iterator(vertex); !edge_iterator.end(); ++edge_iterator)
		{

			Edge * edge = *edge_iterator;

			//get the unit vector corresponding to the direction of the edge
			Point edge_unit_vector = edge->he(0)->target()->point() - edge->he(0)->source()->point();
			edge_unit_vector /= edge_unit_vector.norm();

			//this is the (e x Ne)t from the definition of the edge-based shape operator
			Point e_cross_ne = edge_unit_vector ^ edge_normals[edge->index()];

			//manually assemble the matrix (e x Ne)(e x Ne)t because it's simpler this way
			double a = e_cross_ne.v[0];
			double b = e_cross_ne.v[1];
			double c = e_cross_ne.v[2];

			double edge_based_shape_operator[3][3] = { { a*a, a*b, a*c },
			{ b*a, b*b, b*c },
			{ c*a, c*b, c*c } };

			//multiply all terms by the mean curvature
			for (int i = 0; i < 3; i++)
			{
				for (int ii = 0; ii < 3; ii++)
				{
					edge_based_shape_operator[i][ii] *= mean_curvatures[edge->index()];
				}
			}
			//end of edge-based shape operator calculation

			double np_dot_ne = vertex_normals[vertex->index()] * edge_normals[edge->index()];

			//do the summation part of the definition of the vertex-based shape operator
			for (int i = 0; i < 3; i++)
			{
				for (int ii = 0; ii < 3; ii++)
				{
					vertex_based_shape_operator[i][ii] += np_dot_ne * edge_based_shape_operator[i][ii];
				}
			}

		}

		for (int i = 0; i < 3; i++)
		{
			for (int ii = 0; ii < 3; ii++)
			{
				vertex_based_shape_operator[i][ii] /= 2;
			}
		}

		//calculate k_min, k_min_vector, k_max, k_max_vector from the eigen decomposition
		double eigen_vectors[3][3];
		double eigen_values[3];
		eigen_decomposition(vertex_based_shape_operator, eigen_vectors, eigen_values);

		//the eigen values are stored in order of increasing value (and are all positive), so we take the rightmost two
		k_min[vertex->index()] = eigen_values[1];
		k_max[vertex->index()] = eigen_values[2];

		//the eigen vectors are stored in columns and their column indices correspond to the indices of their eigen values
		k_min_vector[vertex->index()] = Point(eigen_vectors[0][1], eigen_vectors[1][1], eigen_vectors[2][1]);
		k_max_vector[vertex->index()] = Point(eigen_vectors[0][2], eigen_vectors[1][2], eigen_vectors[2][2]);
	}
}

//mean curvature tends to be on the order of e-17 for boundary edges because their dihedral angle is pi
void calculate_mean_curvatures(void)
{
	for (MeshEdgeIterator edge_iterator(pmesh); !edge_iterator.end(); ++edge_iterator)
	{
		Edge * edge = *edge_iterator;

		//first get the edge_vector
		Point edge_vector = edge->he(0)->target()->point() - edge->he(0)->source()->point();

		//directly from the definition
		mean_curvatures[edge->index()] = 2 * edge_vector.norm() * cos(dihedral_angles[edge->index()] / 2);
	}
}

//boundary edges will be assumed to have dihedral angles of pi (flat)
void calculate_dihedral_angles(void)
{
	for (MeshEdgeIterator edge_iterator(pmesh); !edge_iterator.end(); ++edge_iterator)
	{
		Edge * edge = *edge_iterator;

		if (edge->boundary())
		{
			dihedral_angles[edge->index()] = M_PI;
		}
		else
		{
			Point face_0_normal = face_normals[edge->he(0)->face()->index()];
			Point face_1_normal = face_normals[edge->he(1)->face()->index()];

			//no need to normalize because face_normals are unit vectors
			dihedral_angles[edge->index()] = M_PI - acos(constrain(face_0_normal * face_1_normal, 0, 1));
		}

	}

}

void display(void)
{
	SetCamera();
	Render_BoxAndAxes();
	Render_Mesh();
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	win_height = h;
	win_width = w;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	whratio = (double)w / (double)h; 	//A Commonly Suggested Setting: set ratio in gluPerspective to the aspect ratio of the associated viewport
	gluPerspective(60, whratio, axislen*0.01, axislen * 5);
	glMatrixMode(GL_MODELVIEW);	//change back to modelview
	glutPostRedisplay();
}

int main(int argc, char** argv)
{

	if (argc != 2) {
		std::cout << "Usage: " << argv[0] << " input_mesh.mesh \n";
		return 1;
	}

	//Read the obj mesh
	pmesh = new Mesh;
	bool flag = pmesh->readMFile(argv[1]);
	if (!flag) {
		std::cerr << "Fail to read " << argv[1] << "\n";
		return 2;
	}
	

	ComputeBoundingBox();
	face_normals = new Point[pmesh->numFaces()];
	vertex_normals = new Point[pmesh->numVertices()];
	heTrgAngle = new double[pmesh->numEdges() * 2];
	gaussian_curvatures = new double[pmesh->numVertices()];

	mean_curvatures = new double[pmesh->numEdges()];                     //edge-based
	edge_normals = new Point[pmesh->numEdges()];                         //edge-based, obviously
	dihedral_angles = new double[pmesh->numEdges()];                     //edge-based
	k_max = new double[pmesh->numVertices()];                            //vertex-based
	k_max_vector = new Point[pmesh->numVertices()];                      //vertexbased
	k_min = new double[pmesh->numVertices()];                            //vertex-based
	k_min_vector = new Point[pmesh->numVertices()];                      //vertex-based
	e_min = new double[pmesh->numVertices()];                            //vertex-based
	e_max = new double[pmesh->numVertices()];                            //vertex-based
	positive_feature_line_vertices = new bool[pmesh->numVertices()];     //vertex-based
	negative_feature_line_vertices = new bool[pmesh->numVertices()];     //vertex-based
	positive_feature_line_faces = new bool[pmesh->numFaces()];           //face-based
	negative_feature_line_faces = new bool[pmesh->numFaces()];           //face-based
	regular_triangles = new bool[pmesh->numFaces()];                     //face-based
	semi_regular_triangles = new bool[pmesh->numFaces()];                //face-based
	principal_curvature_signs = new bool *[pmesh->numFaces()];
	for (int i = 0; i < pmesh->numFaces(); i++)
	{
		principal_curvature_signs[i] = new bool[6];
	}

	ComputeNormal();
	ComputeGaussianCurvature();


	ComputeConnectedComponentNumber();
	ComputeBoundaryNumber();
	ComputeGenusNumber();

	//feature line detection tasks
	//*****************************************************************
	calculate_dihedral_angles();
	calculate_mean_curvatures();
	calculate_principal_curvatures();
	detect_regular_triangles_and_choose_principal_curvature_signs();
	calculate_extremalities();
	set_principal_curvature_threshold();
	detect_feature_lines();
	process_singular_triangles();
	//*****************************************************************

	findEdges();
	collapseFaces();

	writeToConsole();


	


	//    int number_of_regular_triangles = 0;
	//    cout << "number_of_regular_triangles = " << number_of_regular_triangles << endl;
	//    cout << "total number of triangles: " << pmesh->numFaces() << endl;

	//    for(int i = 0; i < pmesh->numVertices(); i ++)
	//    {
	//        cout << positive_feature_line_vertices[i] << endl;
	//    }

	//    for(int i = 0 ; i < pmesh->numVertices(); i ++)
	//    {
	//        if(e_max[i] < 1e-15 && k_max[i] > 1e-10)
	//            cout << e_max[i] << " " << k_max[i] << endl;
	//    }

	//OpenGL Routines
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(700, 700);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Feature Line Detection");
	MyInit();

	//Register Callback Functions
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMove);
	glutKeyboardFunc(keyboard);
	glutKeyboardUpFunc(keyUp);

	glutMainLoop();

	delete[] heTrgAngle;
	delete[] face_normals;
	delete[] vertex_normals;
	delete[] gaussian_curvatures;
	delete[] feature_line_edges;
	delete[] mean_curvatures;
	delete[] edge_normals;
	delete[] dihedral_angles;
	delete[] k_max;
	delete[] k_max_vector;
	delete[] k_min;
	delete[] k_min_vector;
	delete[] e_min;
	delete[] e_max;
	delete[] positive_feature_line_vertices;
	delete[] negative_feature_line_vertices;
	delete[] positive_feature_line_faces;
	delete[] negative_feature_line_faces;
	delete[] regular_triangles;
	delete[] semi_regular_triangles;
	delete[] collapsed_feature_line_edges;
	for (int i = 0; i < pmesh->numFaces(); i++)
	{
		delete[] principal_curvature_signs[i];
	}
	delete[] principal_curvature_signs;

	delete pmesh;

	return 0;
}


void SetCamera(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(cameraPosition[0], cameraPosition[1], cameraPosition[2], objCenter[0], objCenter[1], objCenter[2], 0, 1, 0);

	glTranslatef(my_Translation[0], my_Translation[1], my_Translation[2]);

	glTranslatef(objCenter[0], objCenter[1], objCenter[2]);	//before doing rotation to the object, move the object center to the origin

	glRotatef(my_Rotation_y, 0.0, 1.0, 0.0);
	glRotatef(my_Rotation_x, 1.0, 0.0, 0.0);

	glTranslatef(-objCenter[0], -objCenter[1], -objCenter[2]);
}


void mouseMove(int x, int y)
{
	double movingScale = axislen / win_height;  // just a scaling factor to make the mouse moving not too sensitive

	double ExtScale = .0025;
	double DiScale = .005;
	double PrinScale = .05;
	
	/* rotation*/

	if (!tweakDi && !tweakEx && !tweakPrinc)
	{
		if (mouseButton == GLUT_LEFT_BUTTON)
		{
			////////////do something////////////////
			my_Rotation_y += x - mousePositionX0;
			my_Rotation_x += y - mousePositionY0;

		}

		/*xy translation */
		if (mouseButton == GLUT_MIDDLE_BUTTON)
		{
			////////////do something ////////////////
			my_Translation[0] += movingScale * (x - mousePositionX0);
			my_Translation[1] -= movingScale * (y - mousePositionY0);
		}

		/* zoom in and out */
		if (mouseButton == GLUT_RIGHT_BUTTON)
		{ // suppose we want to make moving up as zooming out
			my_Translation[2] += movingScale * (y - mousePositionY0);
		}
	}
	else
	{
		if (tweakDi)
		{
			dihedral_angle_upper_threshold -= DiScale * (y - mousePositionY0);

			if (dihedral_angle_upper_threshold < 0)
			{
				dihedral_angle_upper_threshold = 0;
			}
			else if (dihedral_angle_upper_threshold > M_PI)
			{
				dihedral_angle_upper_threshold = M_PI;
			}

			cout << "Dihedral_Angle_Upper_Threshold: " << dihedral_angle_upper_threshold << endl;

		}
		else if (tweakEx)
		{
			extremality_upper_threshold -= ExtScale * (y - mousePositionY0);

			if (extremality_upper_threshold < 1e-5)
			{
				extremality_upper_threshold = 1e-5;
			}
			else if (extremality_upper_threshold > 1)
			{
				extremality_upper_threshold = 1;
			}

			cout << "Extremality_Upper_Threshold: " << extremality_upper_threshold << endl;
		}
		else if (tweakPrinc)
		{
			PrinExpon -= PrinScale * (y - mousePositionY0);

			if (PrinExpon > -2.0)
			{
				PrinExpon = -2.0;
			}
			else if (PrinExpon < -25.0)
			{
				PrinExpon = -25.0;
			}

			principal_curvature_lower_threshold = pow(10, PrinExpon);

			cout << "Principal_Curvature_Lower_Threshold: " << principal_curvature_lower_threshold << endl;
		}

	}
	mousePositionX0 = x;
	mousePositionY0 = y;
	glutPostRedisplay();
}

void mouseClick(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
		mouseButton = GLUT_LEFT_BUTTON;
	else if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
		mouseButton = GLUT_MIDDLE_BUTTON;
	else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
		mouseButton = GLUT_RIGHT_BUTTON;

	mousePositionX0 = x;
	mousePositionY0 = y;
	return;
}



void keyboard(unsigned char key, int x, int y)
{

	switch (key) {

		//selection of normals
	case 's':
		using_face_normals_vertex_normals = !using_face_normals_vertex_normals;
		glutPostRedisplay();
		break;
			
	case 'k':
		render_gaussian_curvature = !render_gaussian_curvature;
		render_feature_lines = false;
		render_principal_curvature = false;
		render_extremalities = false;
		render_edges = false;
		render_collapsed = false;
		glutPostRedisplay();
		break;

	case 'p':
		render_principal_curvature = !render_principal_curvature;
		render_gaussian_curvature = false;
		render_feature_lines = false;
		render_extremalities = false;
		render_edges = false;
		render_collapsed = false;
		glutPostRedisplay();
		break;

	case 'l':
		render_feature_lines = !render_feature_lines;
		render_gaussian_curvature = false;
		render_principal_curvature = false;
		render_extremalities = false;
		render_edges = false;
		render_collapsed = false;
		glutPostRedisplay();
		break;
	
	case 'e':
		render_extremalities = !render_extremalities;
		render_gaussian_curvature = false;
		render_feature_lines = false;
		render_principal_curvature = false;
		render_edges = false;
		render_collapsed = false;
		glutPostRedisplay();
		break;

	case 'w':
		render_edges = !render_edges;
		render_feature_lines = false;
		render_principal_curvature = false;
		render_extremalities = false;
		render_gaussian_curvature = false;
		glutPostRedisplay();
		break;

	case 'c':
		render_collapsed = !render_collapsed;
		render_feature_lines = false;
		render_principal_curvature = false;
		render_extremalities = false;
		render_gaussian_curvature = false;
		glutPostRedisplay();
		break;
	
	case 'r':
		render_gaussian_curvature = false;
		render_principal_curvature = false;
		render_extremalities = false;
		render_edges = false;
		render_feature_lines = false;
		render_collapsed = false;
		glutPostRedisplay();
		break;

	case 'b':
		tweakEx = true;
		tweakDi = false;
		tweakPrinc = false;
		break;
	case 'n':
		tweakEx = false;
		tweakDi = true;
		tweakPrinc = false;
		break;
	case 'm':
		tweakEx = false;
		tweakDi = false;
		tweakPrinc = true;
		break;
		
	case 'd':
		principal_curvature_lower_threshold = pDefault;
		extremality_upper_threshold = eDefault;
		dihedral_angle_upper_threshold = aDefault;
		break;
	default:
		break;
	}
}

void Render_Mesh(){


	if (render_edges || render_collapsed)
	{
		if (render_edges)
		{
			glColor3f(0.0, 1.0, 0.0);
			glBegin(GL_LINES);
			for (MeshEdgeIterator eit(pmesh); !eit.end(); ++eit)
			{
				Edge *e = *eit;

				if (feature_line_edges[e->index()])
				{
					Halfedge *he = e->he(0);
					Point p1 = he->source()->point();
					Point p2 = he->target()->point();

					glVertex3f(p1[0], p1[1], p1[2]);
					glVertex3f(p2[0], p2[1], p2[2]);
				}
			}
			glEnd();
		}

		if (render_collapsed)
		{
			glColor3f(0.0, 1.0, 0.0);
			glBegin(GL_LINES);
			for (MeshEdgeIterator eit(pmesh); !eit.end(); ++eit)
			{
				Edge *e = *eit;

				if (collapsed_feature_line_edges[e->index()])
				{
					Halfedge *he = e->he(0);
					Point p1 = he->source()->point();
					Point p2 = he->target()->point();

					glVertex3f(p1[0], p1[1], p1[2]);
					glVertex3f(p2[0], p2[1], p2[2]);
				}
			}

			glColor3f(1.0, 0.0, 0.0);
			for (unsigned int j = 0; j < addOns.size(); j++)
			{
				Point v = addOns[j].v->point();
				Point c = addOns[j].c;

				glVertex3f(v[0], v[1], v[2]);
				glVertex3f(c[0], c[1], c[2]);
			}
			glEnd();
		}

		if (render_collapsed && render_edges)
		{
			glColor3f(1.0, 1.0, 1.0);
			glEnable(GL_POINT_SIZE);
			glPointSize(10);
			
			glBegin(GL_POINTS);
			for (unsigned int i = 0; i < addOns.size(); i++)
			{
				Point c = addOns[i].c;
				glVertex3f(c[0], c[1], c[2]);
			}
			glEnd();
		}

	}

	else
	{
		glBegin(GL_TRIANGLES);
		for (MeshFaceIterator fit(pmesh); !fit.end(); ++fit)
		{
			Face *f = *fit;
			Halfedge *he = f->he();
			Vertex *v[3];
			v[0] = he->source();
			v[1] = he->target();
			v[2] = he->next()->target();

			Point & fn = face_normals[f->index()];

			if (using_face_normals_vertex_normals)
			{
				glNormal3dv(fn.v);
			}

			for (int j = 0; j < 3; j++)
			{
				if (!using_face_normals_vertex_normals)
				{
					glNormal3dv(vertex_normals[v[j]->index()].v);
				}
				if (render_gaussian_curvature)
				{
					if (gaussian_curvatures[v[j]->index()] < curvatureThreshold[0])
						glColor3d(0, 0.4, 0);
					else if (gaussian_curvatures[v[j]->index()] > curvatureThreshold[1])
						glColor3d(0.4, 0, 0);
					else
						glColor3d(0.7, 0.7, 0.7);
				}
				else if (render_feature_lines)
				{
					glColor3d(10 * positive_feature_line_vertices[v[j]->index()] && (regular_triangles[f->index()] || semi_regular_triangles[f->index()]), 0, 10 * negative_feature_line_vertices[v[j]->index()] && (regular_triangles[f->index()] || semi_regular_triangles[f->index()]));
				}
				else if (render_extremalities)
				{
					glColor3d(10 * e_max[v[j]->index()] * regular_triangles[f->index()], 0, 10 * e_min[v[j]->index()] * regular_triangles[f->index()]);
				}
				else if (render_principal_curvature)
				{
					glColor3d(100 * (k_max[v[j]->index()]), 0, 100 * (k_min[v[j]->index()]));
				}
				else
				{
					glColor3d(0.7, 0.7, 0.7);
				}

				glVertex3dv(v[j]->point().v);
			}
		}
		glEnd();
	}

	/*

	glColor3f(0.7f, 0.7f, 0.7f);
	//traverse all the face and draw them

	if (render_points)
	{
		glBegin(GL_TRIANGLES);
		for (MeshFaceIterator fit(pmesh); !fit.end(); ++fit){
			Face * f = *fit;
			Halfedge * he = f->he();
			Vertex * v[3];
			v[0] = he->source();
			v[1] = he->target();
			v[2] = he->next()->target();

			Point & fn = face_normals[f->index()];
			if (!using_face_normals_vertex_normals)
			{
				glNormal3dv(fn.v);
			}
			for (int i = 0; i < 3; ++i){
				if (using_face_normals_vertex_normals)
					glNormal3dv(vertex_normals[v[i]->index()].v);
				if (render_gaussian_curvature) {
					if (gaussian_curvatures[v[i]->index()] < curvatureThreshold[0])
						glColor3d(0, 0.4, 0);
					else if (gaussian_curvatures[v[i]->index()] > curvatureThreshold[1])
						glColor3d(0.4, 0, 0);
					else
						glColor3d(0.7, 0.7, 0.7);
				}
				if (render_feature_lines)
					glColor3d(10 * positive_feature_line_vertices[v[i]->index()] && (regular_triangles[f->index()] || semi_regular_triangles[f->index()]), 0, 10 * negative_feature_line_vertices[v[i]->index()] && (regular_triangles[f->index()] || semi_regular_triangles[f->index()]));
				//glColor3d(10 * (e_max[v[i]->index()] < 1e-10), 0, 10 * (e_min[v[i]->index()] < 1e-10));
				//glColor3d(100 * e_max[v[i]->index()] * regular_triangles[f->index()], 0, 100 * e_min[v[i]->index()] * regular_triangles[f->index()]);
				//glColor3d(regular_triangles[f->index()], 0, regular_triangles[f->index()]);
				//glColor3d(100 * (abs(e_max[v[i]->index()]) > 1.1) * regular_triangles[f->index()], 0, 100 * (abs(e_min[v[i]->index()]) > 1.1) * regular_triangles[f->index()]);
				//glColor3d(10, 0, 10);
				if (render_principal_curvature)
					//glColor3d(100 * (k_max[v[i]->index()] > 0.01), 0, 100 * (k_min[v[i]->index()] > 0.01));
					glColor3d(100 * (k_max[v[i]->index()]), 0, 100 * (k_min[v[i]->index()]));
				//glColor3d(100 * (k_max[v[i]->index()] > 0.01), 0, 100 * k_min[v[i]->index()]);p
				if (render_extremalities)
					glColor3d(10 * e_max[v[i]->index()] * regular_triangles[f->index()], 0, 10 * e_min[v[i]->index()] * regular_triangles[f->index()]);
				glVertex3dv(v[i]->point().v);
			}
			//cout << e_max[v[0]->index()] << endl;

			//        Edge * e2 = he->edge();
			//        Edge * e0 = he->next()->edge();
			//        Edge * e1 = he->prev()->edge();
			//        
			//        double m0 = mean_curvatures[e1->index()] + mean_curvatures[e2->index()];
			//        double m1 = mean_curvatures[e0->index()] + mean_curvatures[e2->index()];
			//        double m2 = mean_curvatures[e0->index()] + mean_curvatures[e1->index()];
			////        if(m0 > 1e-17)
			////            cout << m0 << endl;
			//        
			//        double threshold = 0.035;
			//        glColor3d(10 * (m0 > threshold), 0, 0);
			//        glVertex3dv(v[0]->point().v);
			//        
			//        glColor3d(10 * (m1 > threshold), 0, 0);
			//        glVertex3dv(v[1]->point().v);
			//        
			//        glColor3d(10 * (m2 > threshold), 0, 0);
			//        glVertex3dv(v[2]->point().v);
		}
	}
	else if (render_edges)
	{
		glBegin(GL_LINES);
		glColor3d(1.0, 1.0, 1.0);

		for (MeshEdgeIterator edge_iterator(pmesh); !edge_iterator.end(); ++edge_iterator)
		{
			Edge * edge = *edge_iterator;

			if (feature_line_edges[edge->index()])
			{
				Halfedge * he = edge->he(0);
				Vertex * source = he->source();
				Vertex * target = he->target();

				glVertex3dv(source->point().v);
				glVertex3dv(target->point().v);
			}
		}

		glColor3d(1, 0, 0);

		//        for (unsigned int k = 0; k < addOns.size(); k++)
		//        {
		//            Point p = addOns[k].v->point();
		//            Point c = addOns[k].c;
		//            
		//            glVertex3f(p[0], p[1], p[2]);
		//            glVertex3f(c[0], c[1], c[2]);
		//        }
		glColor3f(1.0, 0.0, 1.0);
		for (unsigned int i = 0; i < addOns.size(); i++)
		{
			Point c = addOns[i].c;
			Point v = addOns[i].v->point();

			glVertex3f(c[0], c[1], c[2]);
			glVertex3f(v[0], v[1], v[2]);


		}

	}
	glEnd();
	*/
}

//Rendering
void Render_BoxAndAxes() {

	float axiswidth = axislen / 100;

	glMatrixMode(GL_MODELVIEW);

	glColor3f(1, 1, 1);

	glPushMatrix();
	//bounding box
	glTranslatef(objCenter[0], objCenter[1], objCenter[2]);
	glutWireCube(axislen);
	glTranslatef(-axislen / 2, -axislen / 2, -axislen / 2);
	glutSolidSphere(axiswidth*1.5, 10, 10);

	//x-axis
	glColor3f(1, 0, 0);
	glPushMatrix();
	glRotatef(90, 0, 1, 0);
	gluCylinder(obj, axiswidth, axiswidth, axislen, 5, 5);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(axislen, 0, 0);
	glRotatef(90, 0, 1, 0);
	glutWireCone(axiswidth*1.5, axiswidth * 3, 10, 10);
	glPopMatrix();


	//y-axis
	glColor3f(0, 1, 0);
	glPushMatrix();
	glTranslatef(0, axislen, 0);
	glRotatef(-90, 1, 0, 0);
	glutWireCone(axiswidth*1.5, axiswidth * 3, 10, 10);
	glPopMatrix();

	glPushMatrix();
	glRotatef(-90, 1, 0, 0);
	gluCylinder(obj, axiswidth, axiswidth, axislen, 5, 5);
	glPopMatrix();

	//z-axis
	glColor3f(0, 0, 1);
	glPushMatrix();
	glTranslatef(0, 0, axislen);
	glRotatef(-90, 0, 0, 1);
	glutWireCone(axiswidth*1.5, axiswidth * 3, 10, 10);
	glPopMatrix();

	glPushMatrix();
	gluCylinder(obj, axiswidth, axiswidth, axislen, 5, 5);
	glPopMatrix();

	glPopMatrix();
}



void ComputeBoundingBox() {
	objCenter[0] = objCenter[1] = objCenter[2] = 0;
	boxMin[0] = boxMin[1] = boxMin[2] = 1e5;
	boxMax[0] = boxMax[1] = boxMax[2] = -1e5;
	for (MeshVertexIterator vit(pmesh); !vit.end(); ++vit){
		Vertex * v = *vit;
		for (int j = 0; j < 3; ++j){
			float value = v->point()[j];
			objCenter[j] += value;
			if (boxMax[j] < value)
				boxMax[j] = value;
			if (boxMin[j] > value)
				boxMin[j] = value;
		}
	}
	axislen = sqrt((boxMax[2] - boxMin[2])*(boxMax[2] - boxMin[2]) + (boxMax[1] - boxMin[1])*(boxMax[1] - boxMin[1]) + (boxMax[0] - boxMin[0])*(boxMax[0] - boxMin[0]));

	objCenter[0] /= pmesh->numVertices();
	objCenter[1] /= pmesh->numVertices();
	objCenter[2] /= pmesh->numVertices();

	cameraPosition[0] = objCenter[0];
	cameraPosition[1] = objCenter[1];
	cameraPosition[2] = objCenter[2] + axislen*1.5;
}

void ComputeNormal()
{
	//First, compute face normals
	for (MeshFaceIterator fit(pmesh); !fit.end(); ++fit){
		Face * f = *fit;
		Halfedge * he = f->he();
		Halfedge * nhe = he->next();
		Vertex * v1 = he->source();
		Vertex * v2 = he->target();
		Vertex * v3 = nhe->target();


		//had to switch up this line

		//Point nface = (v2->point() - v1->point()) ^ (v3->point() - v2->point());

		Point nface = (v2->point() - v1->point());
		Point nfacebuffer = (v3->point() - v2->point());
		nface = nface ^ nfacebuffer;

		nface /= nface.norm();
		face_normals[f->index()] = nface;
	}

	//Second, compute corner angles
	//Need to set the index for all the halfedges; note that the index for faces and vertices is constructed when reading the mesh file
	for (MeshFaceIterator fit(pmesh); !fit.end(); ++fit){
		Face * f = *fit;
		Halfedge * he[3];
		he[0] = f->he();
		he[1] = he[0]->next();
		he[2] = he[0]->prev();
		double len[3], len2[3];
		for (int i = 0; i<3; ++i)
		{
			len2[i] = (he[i]->target()->point() - he[i]->source()->point()).norm2();
			len[i] = sqrt(len2[i]);
		}
		for (int i = 0; i<3; ++i)
		{
			int ip1 = (i + 1) % 3;
			int ip2 = (i + 2) % 3;
			if (len[i]<1e-8 || len[ip1]<1e-8)
				heTrgAngle[he[i]->index()] = 3.14159265359 / 2;   //to handle the degenerate case
			else
				heTrgAngle[he[i]->index()] = acos((len2[i] + len2[ip1] - len2[ip2]) / (2 * len[i] * len[ip1]));
		}
	}

	//Third, compute vertex normal
	for (MeshVertexIterator vit(pmesh); !vit.end(); ++vit)
	{
		Point sumVNorm(0, 0, 0);
		double sumWeight = 0;
		Vertex * v = *vit;
		for (VertexInHalfedgeIterator heit(v); !heit.end(); ++heit)
		{
			Halfedge * he = *heit;
			Face * f = he->face();
			int he_ind = he->index();
			int f_ind = f->index();

			//had to change sumVNorm += (face_normals[f_ind] * heTrgAngle[he_ind]);

			Point buffer = (face_normals[f_ind] * heTrgAngle[he_ind]);
			sumVNorm += buffer;
			sumWeight += heTrgAngle[he_ind];



		}
		int v_ind = v->index();
		vertex_normals[v_ind] = sumVNorm / sumWeight;
	}

	//compute edge normals from face normals
	for (MeshEdgeIterator edge_iterator(pmesh); !edge_iterator.end(); ++edge_iterator)
	{
		Edge * edge = *edge_iterator;
		if (edge->boundary())
		{
			edge_normals[edge->index()] = face_normals[edge->he(0)->face()->index()];
		}
		else
		{
			edge_normals[edge->index()] = face_normals[edge->he(0)->face()->index()] + face_normals[edge->he(1)->face()->index()];
			edge_normals[edge->index()] /= edge_normals[edge->index()].norm();
		}

	}

}

void ComputeGaussianCurvature(){
	double PI = 3.14159265358979;
	for (MeshVertexIterator vit(pmesh); !vit.end(); ++vit){
		Vertex * v = *vit;
		double angleDeficit = 0;
		for (VertexInHalfedgeIterator vheit(v); !vheit.end(); ++vheit){
			Halfedge * he = *vheit;
			angleDeficit += heTrgAngle[he->index()];
		}
		if (v->boundary())
			gaussian_curvatures[v->index()] = PI - angleDeficit;
		else
			gaussian_curvatures[v->index()] = 2 * PI - angleDeficit;
	}
}

void MyInit()
{
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CCW);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_SMOOTH);
	glPolygonMode(GL_FRONT, GL_FILL);

	obj = gluNewQuadric();	//only for drawing spheres and cones

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	// Create light components
	GLfloat ambientLight0[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat	diffuseLight0[] = { 0.8f, 0.8f, 0.8, 1.0f };
	GLfloat specularLight0[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat position0[] = { cameraPosition[0], cameraPosition[1], cameraPosition[2], 1.0f }; // the light is on the camera position

	// Assign created components to GL_LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight0);
	glLightfv(GL_LIGHT0, GL_POSITION, position0);

	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

void ComputeConnectedComponentNumber(){
	bool * isVisisted = new bool[pmesh->numVertices()];
	for (int i = 0; i < pmesh->numVertices(); ++i)
		isVisisted[i] = false;
	N_c = 0;
	std::vector<Vertex *> vertToVisit;
	for (MeshVertexIterator vit(pmesh); !vit.end(); ++vit){
		Vertex * v = *vit;
		if (!isVisisted[v->index()]) {
			N_c++;
			vertToVisit.push_back(v);
			isVisisted[v->index()] = true;
			while (vertToVisit.size()>0){
				Vertex * cv = vertToVisit.back();
				vertToVisit.pop_back();
				for (VertexVertexIterator vvit(cv); !vvit.end(); ++vvit){
					Vertex * vv = *vvit;
					if (!isVisisted[vv->index()]){
						isVisisted[vv->index()] = true;
						vertToVisit.push_back(vv);
					}
				}
			}
		}
	}
	delete[] isVisisted;
	//std::cout << "Number of connected components: " << N_c << endl;
}

void ComputeBoundaryNumber(){
	bool * isVisisted = new bool[pmesh->numEdges()];
	for (int i = 0; i < pmesh->numEdges(); ++i)
		isVisisted[i] = false;
	N_b = 0;
	for (MeshEdgeIterator eit(pmesh); !eit.end(); ++eit){
		Edge * e = *eit;
		if (e->boundary() && !isVisisted[e->index()]) {
			N_b++;
			Halfedge * he0 = e->he(0);
			isVisisted[he0->edge()->index()] = true;
			Halfedge * he = he0->target()->most_clw_out_halfedge();
			while (he != he0){
				isVisisted[he->edge()->index()] = true;
				he = he->target()->most_clw_out_halfedge();
			}
		}
	}
	delete[] isVisisted;
	//std::cout << "Number of boundaries: " << N_b << endl;
}

void ComputeGenusNumber(){
	if (N_c == 1) {
		int chi = pmesh->numVertices() - pmesh->numEdges() + pmesh->numFaces();
		// chi = 2 - 2g - b;
		N_g = (2 - (chi + N_b)) / 2;
		//std::cout << "Genus number: " << N_g << endl;
	}
}