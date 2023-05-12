/*
 OpenGL models of robot parts, for interactive 3D visualization
 in the backend and frontend.
 
 This code began as omesh/mesh.h, from circa 2006. 
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-03-12 (Public Domain)

*/
#ifndef __AURORA_MODEL_H
#define __AURORA_MODEL_H 1


/** Type used to store vertex numbers. */
typedef int idx_t;


/** A 3D mesh triangle.  Just stores its vertices (0-based numbers) */
class mesh_tri {
public:	
    enum {n=3}; // vertex count
    
	mesh_tri() {vertex[0]=vertex[1]=vertex[2]=-1;}
	mesh_tri(idx_t a,idx_t b,idx_t c) {vertex[0]=a;vertex[1]=b;vertex[2]=c;}
	inline idx_t &operator[] (int i) {return vertex[i];}
	inline idx_t operator[] (int i) const {return vertex[i];}
	inline unsigned int size(void) const {return n;}
private:
	idx_t vertex[n];
};


/** A 3D surface mesh.  All the properties are optional except "coord". */
class mesh {
public:
	/** 3D coordinates of the vertices of the mesh */
	std::vector<osl::Vector3f> coord;
	
	/** Array of mesh triangles.  0-based numbers of vertices. */
	std::vector<mesh_tri> tri;
	/** Return barycentric center of this face */
	osl::Vector3f faceCenter(int f) const {
		return (1.0/3.0)*(coord[tri[f][0]]+coord[tri[f][1]]+coord[tri[f][2]]);
	}
	
	void draw(void) {
	    glBegin(GL_TRIANGLES);
	    for (mesh_tri t : tri) {
	        for (int i=0;i<mesh_tri::n;i++) {
	            glVertex3fv(coord[t[i]]);
	        }
	    }	    
	    glEnd();
	}
};

/** Read a Wavefront .obj file, as exported from Blender with nothing checked. */
mesh readOBJmesh(const std::string &objFilename)
{
    mesh m;
	FILE *f;

	f=fopen(objFilename.c_str(),"r");
	if (f==NULL) throw std::runtime_error("Couldn't open obj file!");
	
	enum {max_line=1000};
	char line[max_line];
	while (0!=fgets(line,max_line,f)) {
		char what[max_line];
		what[0]=0; /* empty */
		sscanf(line,"%s",what); /* get the first thing on the line */
		if (0==strcmp(what,"v")) 
		{ /* vertex coordinates */
			double x,y,z;
			if (4!=sscanf(line,"%s%lf%lf%lf",what,&x,&y,&z))
				throw std::runtime_error("Parse error on 'v' line!");
			m.coord.push_back(osl::Vector3f(x,y,z));
		} 
		else if (0==strcmp(what,"vt")) 
		{ /* vertex texture coordinates */
			double x,y,z=0.0;
			if (3<sscanf(line,"%s%lf%lf%lf",what,&x,&y,&z))
				throw std::runtime_error( "Parse error on 'vt' line!");
		    /*
		    // FIXME: add texcoord support?
			if (m.texcoord2d)
				m.texcoord2d.push_back(osl::Vector2f(x,y));
			*/
		} 
		else if (0==strcmp(what,"f")) 
		{ /* face (triangle or quad) */
			int a,b,c,d=0,n; /* 1-based vertex numbers */
			if (4>sscanf(line,"%s%d%d%d%d",what,&a,&b,&c,&d)) {
				/* also support weird normal-containing syntax */
				if (7>sscanf(line,"%s%d/%d %d/%d %d/%d %d/%d", what,&a,&n,&b,&n,&c,&n,&d,&n)) {
					throw std::runtime_error( "Parse error on 'f' line!");
				}
			}
			if (d==0) { /* triangle */
				mesh_tri t;
				t[0]=a-1; 
				t[1]=b-1; 
				t[2]=c-1; 
				m.tri.push_back(t);
			} else { /* quad: extremely rare, so just tesselate... */
				mesh_tri t1,t2;
				t1[0]=a-1; 
				t1[1]=b-1; 
				t1[2]=c-1; 
				t2[0]=a-1; 
				t2[1]=c-1; 
				t2[2]=d-1; 
				m.tri.push_back(t1);
				m.tri.push_back(t2);
			}
		} 
		/* else just skip unknown lines, comments, etc. */
	}
	fclose(f);
	
	return m;
}


#endif



