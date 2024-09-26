#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>
using namespace std;
#include <chrono>
#include <string>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <random>
#include <list>

#define M_PI 3.14159265358979323846
#define EPSILON 1E-3
#define MAX_RAY_DEPTH 3
#define NRAYS 3
#define FRESNEL false
#define ANTI_ALIASING false
#define PROF_DE_CHAMP false
#define MOTION_BLUR 0

std::default_random_engine generator(10);
 
class Vector {
public:
    explicit Vector(double x = 0, double y = 0, double z = 0) {
        coord[0] = x;
        coord[1] = y;
        coord[2] = z;
    }
    double& operator[](int i) { return coord[i]; }
    double operator[](int i) const { return coord[i]; }
 
    Vector& operator+=(const Vector& v) {
        coord[0] += v[0];
        coord[1] += v[1];
        coord[2] += v[2];
        return *this;
    }
    
    // retourne la norme 2 au carré
    double norm2() const {
        return coord[0]*coord[0] + coord[1]*coord[1] +coord[2]*coord[2];
        // return square(coord[0]) + square(coord[1]) + square(coord[2]);
    }
 
    double coord[3];

    void normalize()
    {
        double n =sqrt(norm2());
        coord[0] /= n;
        coord[1] /= n;
        coord[2] /= n;
    }
    
    // Surcharge de << pour pouvoir afficher un Vector
    friend std::ostream& operator<<(std::ostream& os, const Vector& v) {
        os << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
        return os;
    }

};
 Vector vecteurNul(0.,0.,0.);

Vector operator+(const Vector& a, const Vector& b) {
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const Vector& a, double b) {
    return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator*(double a, const Vector& b) {
    return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const Vector& b) {
    return Vector(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}
Vector cross(const Vector& u, const Vector& v) {
    return Vector(
        u[1]*v[2] - u[2]*v[1],
        u[2]*v[0] - u[0]*v[2],
        u[0]*v[1] - u[1]*v[0]);
}


double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];}

double square(double x){
     return x*x;
}

double uniform(){
    std::uniform_real_distribution<double> distribution(0., 1.);
    return distribution(generator);
}

Vector random_cos(const Vector& N){

    // Creation base orthonormée:
    int i_min = std::min(std::abs(N[0]), std::min(std::abs(N[1]), std::abs(N[2])));
    Vector T1 = N;
    T1[i_min] = 0;
    if (i_min == 0) {
        T1[1] = -N[2];
        T1[2] = N[1];
    } else if (i_min == 1) {
        T1[0] = -N[2];
        T1[2] = N[0];
    } else {
        T1[0] = -N[1];
        T1[1] = N[0];
    }
    T1.normalize();
    Vector T2 = cross(N, T1);

    double r1 = uniform();
    double r2 = uniform();
    double x = cos(2*M_PI*r1)*sqrt(1-r2);
    double y = sin(2*M_PI*r1)*sqrt(1-r2);
    double z = sqrt(r2);

    return z*N + x*T1 + y*T2;
}

 class Ray{
    public:
    Vector O;
    Vector u;
    double time;
    Ray(Vector O, Vector u, double time=0.) : O(O), u(u), time(time){}
};


class Intersection{
    public:
    // Flag pour savoir si il y a eu intersection ou non
    static Vector vecteurNul;
    bool flag;
    int sphere_id;
    Vector P = vecteurNul;
    Vector N = vecteurNul;
    double t;
    Vector albedo = vecteurNul;
    
    ~Intersection() {}

    // constructeur par défaut dans le cas où il n'y a pas d'intersections:
    Intersection() : flag(false){}

    // constructeur dans le cas où il y a effectivement une intersection: 
    Intersection(Vector& P, Vector& N, double& t, Vector albedo) : flag(true), sphere_id(sphere_id), P(P), N(N), t(t), albedo(albedo){}
    Intersection(bool& flag, Vector& P, Vector& N, double& t, Vector albedo) : flag(flag), sphere_id(sphere_id), P(P), N(N), t(t), albedo(albedo){}

};

Vector Intersection::vecteurNul = Vector(0.,0.,0.);

class Geometry {
public:
    Vector albedo;
    bool mirror;
    double n;
    bool hollow;
    Geometry(Vector albedo = Vector(1.,1.,1.), bool mirror = false, double n = 0, bool hollow= false) : albedo(albedo), mirror(mirror), n(n), hollow(hollow) {};
    virtual Intersection intersect(const Ray& r) const = 0;
};




class BoundingBox {
public:
    bool intersect(const Ray& r) const {
        double tx1 = (bbMin[0] - r.O[0]) / r.u[0];
        double tx2 = (bbMax[0] - r.O[0]) / r.u[0];
        double txMin = std::min(tx1, tx2), txMax = std::max(tx1, tx2);
        double ty1 = (bbMin[1] - r.O[1]) / r.u[1];
        double ty2 = (bbMax[1] - r.O[1]) / r.u[1];
        double tyMin = std::min(ty1, ty2), tyMax = std::max(ty1, ty2);
        double tz1 = (bbMin[2] - r.O[2]) / r.u[2];
        double tz2 = (bbMax[2] - r.O[2]) / r.u[2];
        double tzMin = std::min(tz1, tz2), tzMax = std::max(tz1, tz2);
        double tMin = std::max(txMin, std::max(tyMin, tzMin));
        double tMax = std::min(txMax, std::min(tyMax, tzMax));
        return (tMax > 0 && tMax > tMin);
    }
    // bbmin sommet en bas à gauche devant et bbmax sommet en haut à droite derrière
    Vector bbMin, bbMax;
};

class BVH {
public:
    int begin, end;
    BoundingBox bbox;
    BVH* left, * right;
};

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};


class TriangleMesh : public Geometry {
public:
    ~TriangleMesh() {}
	TriangleMesh(const Vector& albedo, bool mirror = false, double n = 0.) : ::Geometry(albedo, mirror, n) {};;
	
    // Mettre nom objet ici
	void readOBJ(const char* obj) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
					col[0] = std::min(1., std::max(0., col[0]));
					col[1] = std::min(1., std::max(0., col[1]));
					col[2] = std::min(1., std::max(0., col[2]));

					vertices.push_back(vec);
					vertexcolors.push_back(col);

				} else {
					sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
					vertices.push_back(vec);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				normals.push_back(vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				} else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					} else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						} else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}

				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					} else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						} else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;								
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							} else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								} else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);
    }

    void load_texture(const char* filename){
        int x,y,c;
        // nombre de canaux
        texture.push_back(stbi_loadf(filename, &x, &y, &c, 3));
        texW.push_back(x);
        texH.push_back(y);
    }


    void scale_and_translate(double scale = 1, const Vector& translation = Vector(0., 0., 0.), int axis = 0, double angle = 0.) {
        for (int i = 0; i < vertices.size(); i++) {
            vertices[i] = vertices[i] * scale + translation;
            }
    }

    BoundingBox compute_BBox(int indice_debut, int indice_fin){
        BoundingBox box;
        box.bbMin = Vector(1E9, 1E9, 1E9);
        box.bbMax = Vector(-1E9, -1E9, -1E9);
        for (int i = indice_debut; i < indice_fin; i++){
            for (int j = 0; j < 3; j++){
                box.bbMin[j] = std::min(box.bbMin[j], vertices[indices[i].vtxi][j]);
                box.bbMax[j] = std::max(box.bbMax[j], vertices[indices[i].vtxi][j]);
                box.bbMin[j] = std::min(box.bbMin[j], vertices[indices[i].vtxj][j]);
                box.bbMax[j] = std::max(box.bbMax[j], vertices[indices[i].vtxj][j]);
                box.bbMin[j] = std::min(box.bbMin[j], vertices[indices[i].vtxk][j]);
                box.bbMax[j] = std::max(box.bbMax[j], vertices[indices[i].vtxk][j]);
            }
        }
        return box;
    }

    void compute_BVH(BVH* noeud, int indice_debut, int indice_fin){
        noeud->begin = indice_debut;
        noeud->end = indice_fin;
        BoundingBox box = compute_BBox(indice_debut,indice_fin);
        noeud->bbox = box;
        noeud->left = NULL;
        noeud->right = NULL;

        Vector taille_boite = box.bbMax - box.bbMin;
        Vector milieu_boite = (box.bbMax+box.bbMin)*0.5;
        int dimension_to_split;
        if (taille_boite[0] > taille_boite[1] && taille_boite[0] > taille_boite[2]){
            dimension_to_split = 0;
        }
        else if (taille_boite[1] > taille_boite[0] && taille_boite[1] > taille_boite[2]){
            dimension_to_split = 1;
        }
        else{
            dimension_to_split = 2;
        }
        int indice_pivot = indice_debut;
        for (int i = indice_debut; i < indice_fin; i++){
            Vector milieu_triangle = (vertices[indices[i].vtxi] + vertices[indices[i].vtxj] + vertices[indices[i].vtxk])*(1./3.);
            if (milieu_triangle[dimension_to_split] < milieu_boite[dimension_to_split]){
                std::swap(indices[i],indices[indice_pivot]);
                indice_pivot++;
        }
    }

    if (indice_fin - indice_debut < 5 || indice_pivot - indice_debut < 2 || indice_fin - indice_pivot < 2){
        return;
    }

    noeud->left = new BVH();
    noeud->right = new BVH();
    compute_BVH(noeud->left, indice_debut, indice_pivot);
    compute_BVH(noeud->right, indice_pivot, indice_fin);
    }

    Intersection intersect(const Ray& r) const {
        double tbest = 1E256;
        double best_alpha, best_beta, best_gamma;
        bool inter_global = false;
        
        // Pas d'intersection
        if (!bvh->bbox.intersect(r)) return Intersection();
    
        std::list<BVH*> l;
        l.push_back(bvh);
    
        int best_i;
        Vector N, P, texture_albedo;
    
        while (!l.empty()) {
            const BVH* current = l.back();
            l.pop_back();
    
            if (current->left) {
                if(current->left->bbox.intersect(r)){
                    l.push_back(current->left);
                };
                if(current->right->bbox.intersect(r)){
                    l.push_back(current->right);
                };
            }
                
            else {
                for (int i = current->begin; i < current->end; i++) {
    
                    const Vector A = vertices[indices[i].vtxi];
                    const Vector B = vertices[indices[i].vtxj];
                    const Vector C = vertices[indices[i].vtxk];
                    Vector u = r.u;
                    Vector O = r.O;
                    Vector e1 = B - A;
                    Vector e2 = C - A;
                    Vector N_local = cross(e1, e2);
                    Vector OA = A - O;
                    Vector PV = cross(OA, u);

                    double prod_UN = dot(u, N_local);
                    double beta = dot(e2, PV) / prod_UN;
                    double gamma = -dot(e1, PV) / prod_UN;
                    double alpha = 1 - beta - gamma;
                    double t_local = dot(OA, N_local) / prod_UN;
    
                    if (beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1 && alpha >= 0 && alpha <= 1 && t_local > 0){
                        inter_global = true;

                        if (t_local < tbest){
                        tbest = t_local;
                        best_i = i;
                        best_alpha = alpha;
                        best_beta = beta;
                        best_gamma = gamma;
                        
                        P = r.O + tbest * r.u;
                        // N = N_local;
                        // N.normalize();

                        N = alpha*normals[indices[i].ni] + beta*normals[indices[i].nj] + gamma*normals[indices[i].nk];
                        N.normalize();
                        }
                    }
                }
            }
        }
    
        if (inter_global) {
            if (texture.size() == 0 || indices[best_i].group >= texture.size()) {
                texture_albedo = albedo;
            }
            else {
                Vector UVP = uvs[indices[best_i].uvi] * best_alpha + uvs[indices[best_i].uvj] * best_beta + uvs[indices[best_i].uvk] * best_gamma;
                UVP[0] = fabs(UVP[0]);
                UVP[0] = UVP[0] - floor(UVP[0]);
                UVP[1] = fabs(UVP[1]);
                UVP[1] = UVP[1] - floor(UVP[1]);
                UVP[1] = 1 - UVP[1];
                UVP = UVP * Vector(texW[indices[best_i].group], texH[indices[best_i].group], 0);
                int u = std::min((int)UVP[0], texW[indices[best_i].group] - 1);
                int v = std::min((int)UVP[1], texH[indices[best_i].group] - 1);
                int indice_pixel = v * texW[indices[best_i].group] + u;
                texture_albedo = Vector(texture[indices[best_i].group][indice_pixel * 3], texture[indices[best_i].group][indice_pixel * 3 + 1], texture[indices[best_i].group][indice_pixel * 3 + 2]);
            }
        }
    
        return Intersection(inter_global, P, N, tbest, texture_albedo);
    }

    // Face group, indice de la texture
	// Chaque triange contient 3 indices qui nous intéressent vtxi, vtxj, vtxk
    std::vector<float*> texture;
    std::vector<int> texW, texH;

    BVH* bvh;

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
};


class Sphere : public Geometry{
    public:
    Vector C;
    double R;
    Vector velocity;

    Sphere(Vector C, double R, Vector albedo, bool mirror, double n, bool hollow, Vector velocity = vecteurNul) : C(C), R(R), velocity(velocity), ::Geometry(albedo, mirror, n, hollow){}

    Intersection intersect(const Ray& r) const {
        Vector C_actuel = C + velocity*r.time;
        double a =1;
        double b = 2*dot(r.u, r.O - C_actuel);
        double c = (r.O - C_actuel).norm2() -R*R;
        double delta = b*b -4*a*c;

        if (delta <0) return Intersection();
        
        delta = sqrt(delta);
        double t2 = (-b+delta)/(2*a);

        // L'objet est derrière la caméra
        if (t2<0) return Intersection();
        double t1 = (-b-delta)/(2*a);
        // Il n'y a qu'une intersection devant: t2
        if (t1 < 0){
            Vector P = r.O + t2*r.u;
            Vector N = (P-C);
            N.normalize();
            return Intersection(P, N, t2, this->albedo);
        }
        // Il y a 2 intersections, t1 est la plus proche
        else {
            Vector P = r.O + t1*r.u;
            Vector N = (P-C);
            N.normalize();
            return Intersection(P, N, t1, this->albedo);
        }
        }
};

class Scene{
    public:
    std::vector<Geometry*> liste_spheres;
    Scene(){}
    void addSphere(Geometry* S){liste_spheres.push_back(S);}
    Vector L;
    double light_intensity;

    bool intersection_scene(const Ray& r, Intersection& inter, int& i_intersection){
        bool intersection_globale = false;
        double best_t = 1E99;
        i_intersection = 0;
        for (int i = 0; i < liste_spheres.size(); i++){
            //std::cout << "i = " << i << std::endl;
            Intersection inter2 = liste_spheres[i]->intersect(r);
            if (inter2.flag){
                intersection_globale = true;
                if (inter2.t < best_t){
                    best_t = inter2.t;
                    inter = inter2;
                    i_intersection = i;
                }
            }
        }
        return intersection_globale;
        }
  
    int is_visible(const Intersection& inter){
        Vector PL = L - inter.P;
        double d = sqrt(PL.norm2());
        PL.normalize();
        for (int i = 1; i < liste_spheres.size(); i++){
            Intersection inter2 = liste_spheres[i]->intersect(Ray(inter.P+ EPSILON*PL, PL));
            if (inter2.flag && inter2.t >0 && inter2.t < d){
                // Il y a un obstacle entre l'objet et la lumière, donc l'objet est dans l'ombre
                return 0;
            }
        }
        // Il n'y a pas d'obstacle entre l'objet et la lumière, donc l'objet est éclairé
        return 1;
    }

    Vector getColor(const Ray& ray , int nb_rebond, bool was_diffuse_interaction = false){
        if (nb_rebond <= 0){
            return vecteurNul;}
        
        Intersection inter;
        int i;
        if (intersection_scene(ray, inter, i)){
            if(i==0){
                if(was_diffuse_interaction){
                    return vecteurNul;
                }
                else{
                    return Vector(1.,1.,1.)*(light_intensity/(4*square(M_PI*dynamic_cast<const Sphere*>(liste_spheres[i])->R)));
                }
            }
                    

            else if (liste_spheres[i]->mirror){
                // Reflection 
                
                 Vector wr = ray.u - 2*dot(ray.u, inter.N)*inter.N;
                 wr.normalize();
                 Ray reflected_ray = Ray(inter.P+ EPSILON*wr, wr);
                 return getColor(reflected_ray, nb_rebond-1);
            }
            else if (liste_spheres[i]->n){
                // Refraction
                double n1,n2;
                Vector N;


                if (dot(ray.u, inter.N) < 0){
                    // On rentre dans la sphère
                    n1 = 1;
                    n2 = liste_spheres[i]->n;
                    N = inter.N;
                }
                else{
                    // On sort de la sphère
                    n1 = liste_spheres[i]->n;
                    n2 = 1;
                    N = -1*inter.N;
                }

                if (liste_spheres[i]->hollow){
                    ;
                    std::swap(n1,n2);
                    ;
                }

                // Pour éviter de recaculer a chaque fois <wi,N>
                double prod = dot(ray.u, N);

                double int_racine = 1 - (n1/n2) * (n1/n2) * (1 - prod*prod);

                Vector wr = ray.u - 2*dot(ray.u, N)*N;
                wr.normalize();
                Ray reflected_ray = Ray(inter.P+ EPSILON*N, wr);
                
                if (int_racine < 0){
                    // Reflection totale
                    return getColor(reflected_ray, nb_rebond-1);
                }
                else{
                    // Réflection et transmission
                    // wt = wt T + wt N
                    Vector wt = n1/n2*(ray.u - prod*N) - N*sqrt(int_racine);
                    wt.normalize();
                    Ray transmitted_ray = Ray(inter.P - EPSILON*N, wt);

                    // Avec coeff Fresnels:
                    double k0 = (n1 - n2)*(n1 - n2)/((n1 + n2)*(n1 + n2));
                    double R = k0 + (1 - k0)*std::pow(1 - abs(dot(ray.u, N)), 5);
                    double T = 1 - R;
                    if (uniform() < R && FRESNEL){
                        return getColor(reflected_ray, nb_rebond-1);
                    }

                    // Sans coeff Fresnels:
                    return getColor(transmitted_ray, nb_rebond-1);
                }

            }
            else {
                // Surface Diffuse
                Vector Lo = vecteurNul;

                // Eclairage direct
                Vector wi = L-inter.P;
                double d2 = wi.norm2();
                wi.normalize();
                int visible = is_visible(inter);
                // Lo += visible * light_intensity/(4*M_PI*M_PI*d2) * liste_spheres[i]->albedo * std::max(0., dot(wi, inter.N));
                Lo += visible * light_intensity/(4*M_PI*M_PI*d2) * inter.albedo * std::max(0., dot(wi, inter.N));

                // Eclairage indirect
                was_diffuse_interaction = true;
                Ray ray_indirect(inter.P+EPSILON*inter.N, random_cos(inter.N));
                // Lo += albedo_tex * getColor(ray_indirect, nb_rebond-1, true);
                // Lo += liste_spheres[i]->albedo * getColor(ray_indirect, nb_rebond-1, true);
                Lo += inter.albedo * getColor(ray_indirect, nb_rebond-1, true);
                return Lo;
                 }
            }

        // std::cout << "Aucune intersection" << std::endl;
        return vecteurNul;        
        }
};


int main() {
    auto start = std::chrono::high_resolution_clock::now();
    int W = 512;
    int H = 512;

    // MC converge en 1/racine de N, donc pour réduire le bruit par 10, il faut multiplier par 100 le nb de rayons
    Scene scene;

    // Source de lumière
    scene.L = Vector(-10,20,60);
    scene.light_intensity = 5E10;

    // Boule lumineuse 
    scene.addSphere(new Sphere(scene.L, 10., Vector(1.,1.,1.), false, 0, false));

    // Sphère 1 (a gauche) Full
    // scene.addSphere(Sphere(Vector(-20,0,0), 10., Vector(0.5,0.5,1), 1.5));

    // Sphère 2 (au centre)
    // scene.addSphere(new Sphere(Vector(0,0,0), 10., Vector(1.,1.,1), false, 0, false, 20*Vector(0,1.,0.)));

    // Sphère légèrement out of focus devant
    // scene.addSphere(new Sphere(Vector(-10,0,20), 2., Vector(1.,1.,1), false, 1.5, false));
    
    // scene.addSphere(new Sphere(Vector(5,5,5), 4., Vector(1,1,1), false, 1.5, false));


    // Sphère légèrement out of focus derriere
    // scene.addSphere(new Sphere(Vector(15,20,-20), 7., Vector(0.,0.,1.), false, 0, false));

    // Sphère 3 (a droite) Hollow
    // scene.addSphere(Sphere(Vector(20,0,0), 10., Vector(0.5,0.5,1), 1.5));
    // scene.addSphere(Sphere(Vector(20,0,0), 9.8, Vector(0.5,0.5,1), 1.5, true));

    // Sol (bleu)
    scene.addSphere(new Sphere(Vector(0,-1000,0), 990., Vector(0.1, 0.1, 1), false, 0, false));

    // Mur du fond (vert)
    scene.addSphere(new Sphere(Vector(0,0,-1000), 940., Vector(0, 0.4, 0), false, 0, false));

    // Plafond (rouge)
    scene.addSphere(new Sphere(Vector(0,1000,0),940., Vector(1,0,0), false, 0, false));

    // Mur de derriere (rose)
    scene.addSphere(new Sphere(Vector(0,0,1000), 940., Vector(1, 0.753, 0.796), false, 0, false));
    
    // Mur de droite (jaune)
    scene.addSphere(new Sphere(Vector(1000,0,0), 940., Vector(1., 1., 0.), false, 0, false));

    // Mur de gauche (magenta)
    scene.addSphere(new Sphere(Vector(-1000,0,0), 940., Vector(1., 0., 1.), false, 0, false));

    // Caméra
    Vector Camera = Vector(0,0,55);

    // Base orthonormée pour la caméra:
    Vector cameraUp(0,1,0);  
    Vector cameraDir(0,0,-1); // vers le fond de l'image
    Vector cameraRight = cross(cameraDir, cameraUp);
    double rotAngle = 0 * M_PI/180;
    
    // Afin d'éviter de recalculer dans la boucle ensuite
    double cos_theta = cos(rotAngle);
    double sin_theta = sin(rotAngle);

    cameraDir = Vector(sin_theta, 0, -(1-cos_theta)); 

    Camera += cameraDir * 40;  // taille du cercle sur lequel tourne la caméra


    TriangleMesh m(Vector(1,0.1,0.1));
    m.readOBJ("cat.obj");
    m.load_texture("cat_diff.png");
    m.scale_and_translate(0.6, Vector(0,-10,0));
    m.bvh = new BVH();
    m.compute_BVH(m.bvh, 0, m.indices.size());
    scene.addSphere(&m);

    // Conversion de alpha en radians
    double fov = 60*M_PI/180.;
    double d = W/(2.*tan(fov/2.));

    std::vector<unsigned char> image(W*H * 3, 0);
#pragma omp parallel for schedule(dynamic,1)
// 256 rayons : 1 minute
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            Vector color(0.,0.,0.);
            for (int k = 0; k < NRAYS; k++){

                // Vecteur directeur du rayon
                Vector u(j-W/2+0.5, -i+H/2.-0.5, -d);
                Vector u_copy = u;
                // Rotation matrix in 2D (x,z):
                // R = [cos(theta)  -sin(theta)]
                //     [sin(theta)   cos(theta)]
                u[0] =  u[0]*cos_theta + u[2]*sin_theta;
                u[2] = -u[0]*sin_theta + u[2]*cos_theta;
                // u = u[2]* cameraDir + u[0]*cameraRight + u[1]*cameraUp;
                u.normalize();
                // Rayon qui part de la caméra vers le pixel (i,j)
                Ray r = Ray(Camera,u);

                // Avec MOTION BLUR
                // Ray r = Ray(Camera,u, MOTION_BLUR*uniform());

                if (ANTI_ALIASING || PROF_DE_CHAMP){
                    // ANTI-ALIASING:
                    double r1 = uniform(), r2 = uniform();
                    double r3 = sqrt(-2*log(r1));
                    // 99% de la gausienne tombe dans 3*sigma, donc ici 99% tombe dans la taille du pixel  
                    double g1 = r3 * cos(2*M_PI*r2)*0.33;
                    double g2 = r3 * sin(2*M_PI*r2)*0.33;
                    Vector u_anti_aliasing = u_copy + Vector(g1,g2,0.);
                    u_anti_aliasing.normalize();
                    if (ANTI_ALIASING){
                        Ray r =  Ray(Camera,u_anti_aliasing);
                    }
                    else{
                        // PROFONDEUR DE CHAMP
                        double foc = 55.;
                        double x = (uniform() - 0.5) * 10;
                        double y = (uniform() - 0.5) * 10;

                        Vector target = Camera + foc * u_anti_aliasing;
                        Vector Camera_prof_de_champ = Camera + Vector(x, y, 0.);
                        Vector u_prof_de_champ = (target - Camera_prof_de_champ);
                        u_prof_de_champ.normalize();
                        r = Ray(Camera_prof_de_champ,u_prof_de_champ);
                    }

                }

                color += scene.getColor(r,MAX_RAY_DEPTH);
            }
                image[(i*W + j) * 3 + 0] = std::min(255.,std::pow(color[0]/NRAYS,0.45));   // RED
                image[(i*W + j) * 3 + 1] = std::min(255.,std::pow(color[1]/NRAYS,0.45));  // GREEN
                image[(i*W + j) * 3 + 2] = std::min(255.,std::pow(color[2]/NRAYS,0.45));  // BLUE
            


         }
    }
    // stbi_write_png("mesh_test.png", W, H, 3, &image[0], 0);
    stbi_write_png("images/mesh_2.png", W, H, 3, &image[0], 0);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << duration.count() << " s" << std::endl;
    return 0;
};