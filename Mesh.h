#include "Stream.h"
#include "NxPhysics.h"
#include "NxCooking.h"
#include <vector>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "rhostring.hpp"
#include "bibutil.h"

// Essa estruturinha Point eu fiz meio que na correria, só pra ordenar os vértices pra calcular o convex hull
// O dia que eu tiver mais tempo vou incorporá-la a estrutura VERT, que é a que representa os vértices na bibutil
// Outra coisa interessante seria fazer uma bibutil pra PhysX, usando as estruturas da biblioteca. Enfim...
	struct Point {
		float x, y, z;

	    public: 
			Point() { x = 0.0; y = 0.0; z = 0.0; } 
			Point(float x,  float y, float z) {
				this->x = x;
				this->y = y;
				this->z = z;
			}
	}; 
	// second, we define the 
	bool minX (const Point& left, const Point& right) {
		if (left.x < right.x) return true; 
		return false; 
	}

	bool minY (const Point& left, const Point& right) {
		if (left.y < right.y) return true; 
		return false; 
	}

	bool minZ (const Point& left, const Point& right) {
		if (left.z < right.z) return true; 
		return false; 
	}

	bool maxX (const Point& left, const Point& right) {
		if (left.x > right.x) return true; 
		return false; 
	}

	bool maxY (const Point& left, const Point& right) {
		if (left.y > right.y) return true; 
		return false; 
	}

	bool maxZ (const Point& left, const Point& right) {
		if (left.z > right.z) return true; 
		return false; 
	}

class Mesh {

	public:
	
	Mesh() {;}
	Mesh(std::string fname, NxVec3 pos, float mass, bool dynamic,NxPhysicsSDK *gPhysicsSDK, NxScene *gScene);

	std::vector <NxVec3> Mesh::getConvexHull(OBJ *obj);
	NxActor* CreateConvexMesh(NxVec3& pos, float density, std::vector <NxVec3> points,NxPhysicsSDK *gPhysicsSDK, NxScene *gScene);
	NxActor* CreateTriangleMesh(NxVec3 pos,NxPhysicsSDK *gPhysicsSDK, NxScene *gScene);
	
	//Estrutura do corpo rígido da PhysX (Ator)
	NxActor *actor;

	//E se optar por 1 actor por sub-mesh
	std::vector <NxActor*> actors;

	//Estrutura que guarda as informações geométricas do objeto (ver bibutil.h para entender)
	OBJ *obj;

	//Indices do convex Hull
	std::vector <NxVec3> pts;
};

Mesh::Mesh(std::string fname, NxVec3 pos, float mass, bool dynamic,NxPhysicsSDK *gPhysicsSDK, NxScene *gScene){
	// 1) Para carregar todo um objeto 3D como um ator dinâmico, use (ou adapte) a função CreateConvexMesh
	// Para usá-la você antes precisa carregar o .obj em uma estrutura OBJ (vide bibutil.h) com a função
	// CarregaObjeto
	obj = CarregaObjeto(fname,true);
	
	// Os objetos podem ter suas coordenadas de mundo muito grandes (ou muito pequenas). Para escalá-las, use 
	// a função escalaObjeto 
	escalaObjeto(obj,1.5);
	
	if (dynamic == true) {
	// Agora, você precisa calcular o convex hull, que será a malha convexa do ator dinâmico. A função a seguir
	// retorna os vértices desse convexHull, para uso na próxima função
	pts = getConvexHull(obj);
	
	// Enfim, o ator dinâmico é criado, usando-se os vértices do convex hull retornado anteriormente
	actor = CreateConvexMesh(pos, mass, pts,gPhysicsSDK,gScene);
	actor->setName("mesh");
	}
	else {
		actor = CreateTriangleMesh(pos,gPhysicsSDK,gScene);
		actor->setName("mesh");
	}
	
	// Se precisarem rotacionar o ator, eis um exemplo de como: (tá hard-coded, FAÇAM UM MÉTODO!!!)
	NxQuat q;
	q.fromAngleAxis(180, NxVec3(0,1,0)); //angulo e eixo 180º em Y
	actor->setGlobalOrientationQuat(q);

	// Neste exemplo, estou usando o atributo "name" dos atores para indicar na função RenderActors que eu não quero
	// desenhar aqueles que são do tipo "mesh". Senão a malha do convex hull será desenhada azulzinha
	//actor->setName("mesh");
	//actors.push_back(actor);

};

std::vector <NxVec3> Mesh::getConvexHull(OBJ *obj){

	std::vector <NxVec3> pts;

	std::vector <Point> allVerts;
	std::vector <Point> sortedByX;
	std::vector <Point> sortedByY;
	std::vector <Point> sortedByZ;
	float accx = 0.0, accy = 0.0, accz = 0.0;
	
	int totalVertices = 0;
	std::vector <float> percents;
	std::vector <float> areas;

	 for (int i=0; i<obj->meshes.size(); i++) {
		totalVertices += obj->meshes[i]->vertices.size();

		

		
		for(int j=0; j<obj->meshes[i]->vertices.size(); j++){
			allVerts.push_back(Point(obj->meshes[i]->vertices[j].x,obj->meshes[i]->vertices[j].y,obj->meshes[i]->vertices[j].z));
		}
		
		
	

		//areas.push_back((accx + accy + accz) / 3 );
	 }

	 int size = 256;
	 if (totalVertices < 256)
		 size = totalVertices;

	 sort(allVerts.begin(), allVerts.end(), minX);
	 sortedByX = allVerts;
	 accx = fabs(allVerts[allVerts.size()-1].x - allVerts[0].x);

	 sort(allVerts.begin(), allVerts.end(), minY);
	 sortedByY = allVerts;
	 accy = fabs(allVerts[allVerts.size()-1].y - allVerts[0].y);

	 sort(allVerts.begin(), allVerts.end(), minZ);
	 sortedByZ = allVerts;
	 accz = fabs(allVerts[allVerts.size()-1].z - allVerts[0].z);
	 
	 std::cout << "-- " << accx << " " << accy << " " << accz << " -- " << "\n";

	 float x = (float) size/(accx + accy + accz);
	 int sliceX = accx * x;
	 int sliceY = accy * x;
	 int sliceZ = accz * x;

	 int total = sliceX + sliceY + sliceZ;

	 if (total < size)
		 sliceX += size -total;

	 //std::cout << "x = " << x << " sliceX = " << sliceX << " sliceY = " << sliceY << " sliceZ = " << sliceZ << " total =  " << sliceX+sliceY+sliceZ << "\n";
	 //std::cout << "Total de vertices = " << totalVertices << "\n";
	
	
	 int shiftX = allVerts.size()/sliceX;
	 int shiftY = allVerts.size()/sliceY;
	 int shiftZ = allVerts.size()/sliceZ;

	 //std::cout << "ShiftX = " << shiftX << " ShiftY = " << shiftY << " ShiftZ = " << shiftZ << "\n";

	 int i = 0;
	 while (i < sliceX){
		 pts.push_back(NxVec3(sortedByX[i].x,sortedByX[i].y,sortedByX[i].z));
		 pts.push_back(NxVec3(sortedByX[allVerts.size()-1-i].x,sortedByX[allVerts.size()-1-i].y,sortedByX[allVerts.size()-1-i].z));
		 i+=2;
	 }

	 i = 0;
	 while (i < sliceY){
		 pts.push_back(NxVec3(sortedByY[i].x,sortedByY[i].y,sortedByY[i].z));
		 pts.push_back(NxVec3(sortedByY[allVerts.size()-1-i].x,sortedByY[allVerts.size()-1-i].y,sortedByY[allVerts.size()-1-i].z));
		 i+=2;
	 }

	 i = 0;
	 while (i < sliceZ){
		 pts.push_back(NxVec3(sortedByZ[i].x,sortedByZ[i].y,sortedByZ[i].z));
		 pts.push_back(NxVec3(sortedByZ[allVerts.size()-1-i].x,sortedByZ[allVerts.size()-1-i].y,sortedByZ[allVerts.size()-1-i].z));
		 i+=2;
	 }

	 //std::cout << "pts.size() = " << pts.size() << "\n";
	 pts.resize(size);
	 
		

	//std::cout << "N of verts " << sortedByX.size() << " Slice = " << slices << " Number of points: " << pts.size() << "\n";

	return pts;
}

NxActor* Mesh::CreateConvexMesh(NxVec3& pos, float mass, std::vector <NxVec3> points,NxPhysicsSDK *gPhysicsSDK, NxScene *gScene) {
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	NxVec3* verts = (NxVec3*) malloc( points.size() * sizeof(NxVec3));
	
	for(int i=0; i< points.size(); i++) {
		verts[i] = points[i];	
	}

	int	vertNb;
	if (points.size() < 256)
		vertNb = points.size();
	else vertNb = 255;
	
	// Create descriptor for convex mesh
	NxConvexMeshDesc *convexDesc = new NxConvexMeshDesc();
	convexDesc->numVertices			= vertNb;
	convexDesc->pointStrideBytes	= sizeof(NxVec3);
	convexDesc->points				= verts;
	convexDesc->flags				= NX_MF_HARDWARE_MESH;

	
	// The actor has one shape, a convex mesh
	NxConvexShapeDesc convexShapeDesc;
	
	// Fazendo o "cooking" da mesh	em memória
	NxInitCooking();

	// Em memória
	MemoryWriteBuffer buf;
	bool status = NxCookConvexMesh(*convexDesc, buf);
	
	if (status)
	{
		convexShapeDesc.meshData =  gPhysicsSDK->createConvexMesh(MemoryReadBuffer(buf.data));
		
	}
	else
	{
		std::cout << "nao deu essa mesh\n";
		return NULL;
		assert(false);
	}
	NxCloseCooking();


	convexShapeDesc.localPose.t = NxVec3(0,0,0);
	actorDesc.shapes.pushBack(&convexShapeDesc);

	if (mass)
	{
		actorDesc.body = &bodyDesc;
		//Isso foi uma forma alternativa pra se calcular a densidade: aproximei a uma esfera
		//Ainda vou pensar numa maneira mais inteligente...
		NxSphere sphere;
		NxComputeSphere(sphere,vertNb,verts);
		//std::cout << "Raio da esfera! " << sphere.radius << "\n";
		actorDesc.density = NxComputeSphereDensity(sphere.radius,mass);
	}
	else
	{
		actorDesc.body = NULL;
	}
	actorDesc.globalPose.t = pos;
	return gScene->createActor(actorDesc);
}

NxActor* Mesh::CreateTriangleMesh(NxVec3 pos,NxPhysicsSDK *gPhysicsSDK, NxScene *gScene)
{
	
	NxVec3 *verts;

	int totalVertices = 0;
	int totalFaces = 0;
	std::vector <NxVec3> allVerts;
	std::vector <NxU32> indVerts;
	
	

	 for (int i=0; i<obj->meshes.size(); i++) {
		totalVertices += obj->meshes[i]->vertices.size();
		for(int j=0; j<obj->meshes[i]->vertices.size(); j++){
			allVerts.push_back(NxVec3(obj->meshes[i]->vertices[j].x,obj->meshes[i]->vertices[j].y,obj->meshes[i]->vertices[j].z));
		}
	 }
	verts = (NxVec3 *) malloc(sizeof(NxVec3)*totalVertices);
	for(int i=0; i<allVerts.size(); i++)
		verts[i] = allVerts[i];

	std::cout << obj->meshes.size() << "\n";
	for (int i=0; i<obj->meshes.size(); i++) {
		totalFaces += obj->meshes[i]->faces.size();
		std::cout << "mesh " << i << "\n";
		for(int j=0; j<obj->meshes[i]->faces.size(); j++){
			indVerts.push_back(obj->meshes[i]->faces[j].vert[0]);
			indVerts.push_back(obj->meshes[i]->faces[j].vert[1]);
			indVerts.push_back(obj->meshes[i]->faces[j].vert[2]);
			
		}
	 }

	std::cout << "totalFaces = " << totalFaces << "\n";

	NxU32 *indices;
	indices = (NxU32 *) malloc(sizeof(NxU32)*totalFaces*3);

	for(int i=0; i<indVerts.size(); i++) {
		indices[i] =  (NxU32) indVerts[i];
	}
	 

	// Create descriptor for triangle mesh
	 NxTriangleMeshDesc* triangleMeshDesc = NULL;
    if (!triangleMeshDesc)
	{
		triangleMeshDesc	= new NxTriangleMeshDesc();
		assert(triangleMeshDesc);
	}
	triangleMeshDesc->numVertices			= totalVertices;
	triangleMeshDesc->pointStrideBytes		= sizeof(NxVec3);
	triangleMeshDesc->points				= verts;
	triangleMeshDesc->numTriangles			= totalFaces;
	triangleMeshDesc->flags					= 0;
	triangleMeshDesc->triangles				= indices;
	triangleMeshDesc->triangleStrideBytes	= 3 * sizeof(NxU32);
	
	
	//Alternative:	see NxMeshFlags
	//triangleMeshDesc->flags				= NX_MF_16_BIT_INDICES
	//triangleMeshDesc->triangles			= indices16;
	//triangleMeshDesc->triangleStrideBytes	= 3 * sizeof(NxU16);

	// The actor has one shape, a triangle mesh
	NxInitCooking();
	MemoryWriteBuffer buf;

	bool status = NxCookTriangleMesh(*triangleMeshDesc, buf);
	NxTriangleMesh* pMesh;
	if (status)
	{
		pMesh = gPhysicsSDK->createTriangleMesh(MemoryReadBuffer(buf.data));
	}
	else
	{
		assert(false);
		pMesh = NULL;
	}
	NxCloseCooking();

	
	// Create TriangleMesh above code segment.

	NxTriangleMeshShapeDesc tmsd;
	tmsd.meshData		= pMesh;
	tmsd.userData		= triangleMeshDesc;
	tmsd.localPose.t	= NxVec3(0, 0, 0);
	tmsd.meshPagingMode = NX_MESH_PAGING_AUTO;
	
	NxActorDesc actorDesc;
	NxBodyDesc  bodyDesc;
	
	assert(tmsd.isValid());
	actorDesc.shapes.pushBack(&tmsd);
	//Dynamic triangle mesh don't be supported anymore. So body = NULL
	actorDesc.body			= NULL;		
	actorDesc.globalPose.t	= pos;

	if (pMesh)
	{
		// Save mesh in userData for drawing
		pMesh->saveToDesc(*triangleMeshDesc);
		//
		assert(actorDesc.isValid());
		NxActor *actor = gScene->createActor(actorDesc);
		assert(actor);

	

		return actor;
	}



	return NULL;
}

