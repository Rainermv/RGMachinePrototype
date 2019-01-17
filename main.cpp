/*************************************************************
Projeto inicial com a PhysX 2.8.4 � F�sica para Jogos 2011-2
Corpos R�gidos

C�digo base, iniciado utilizando o exemplo abaixo:
page : http://mmmovania.blogspot.com/2011/04/getting-started-with-nvidia-physx.html

e NxBoxes da documenta��o da PhysX

Adaptado inicialmente por:
Rossana B. Queiroz -- 23/10/2011
*************************************************************/

// TRABALHO MAQUINA VIRTUAL

#include <iostream>
#include <GL/glut.h>

#include <vector>

#include <NxPhysics.h>

//Conjunto de Rotinas para desenho de primitivas, c�digo que vem junto com os exemplos da PhysX
#include "DrawObjects.h"

//Classe para a manipula�ao de malhas
#include "Mesh.h"

//Classe auxiliar para a cria��o de juntas
#include "Joints.h"

using namespace std;

const int	WINDOW_WIDTH=1024, 
WINDOW_HEIGHT=768;

#define MAX_PATH 512

//Globais com objetos das principais classes da PhysX
static NxPhysicsSDK* gPhysicsSDK = NULL;
NxScene* gScene = NULL;
NxActor* groundPlane; 
NxActor* box; 
NxReal myTimestep = 1.0f/60.0f;

//Globais para navega��o com o Mouse
int oldX=0, oldY=0;
float rX=15, rY=0;
float fps=0;
int startTime=0;
int totalFrames=0;
int state =1 ;
float dist=-30;

//CorpoSelecionado
NxActor *gSelectedActor;

//Globais para a aplica��o de for�a
NxVec3	gForceVec(0,0,0);
NxReal	gForceStrength	= 2000;
NxVec3  gForcePos;
bool isAppyingForce = false;

// The joints
vector <NxFixedJoint> *fixed_joints;
vector <NxRevoluteJoint> *rev_joints;
vector <NxSphericalJoint> *spherical_joints;
vector <NxPulleyJoint> *pulley_joints;

// Motor for pulley joint
vector <NxMotorDesc> motors;

//NxFixedJoint* fixedJoint = NULL;
//NxRevoluteJoint* revJoint = NULL;
//NxSphericalJoint* sphericalJoint = NULL;
//NxPulleyJoint* pulleyJoint = NULL;



//Classezinha auxiliar para desenho da seta quando atira esferas 
class arrow{
public:
	arrow() { length = 3; }
	NxVec3 ori;
	NxVec3 dir;
	float length;
};
//Globais para o desenho da seta
arrow seta;
bool desenhaSeta = false;


//Alguns cabe�alhos de fun��es -- Essas fun��es s�o fun��es exemplo de cria��o de cada tipo de 
//Corpo r�gido que a PhysX suporta. Voc� deve criar fun��es que generalizem os principais par�metros
//Para que possa criar objetos diferenciados na cena
NxActor*  CreateSphere2(NxVec3 pos, float radius, float dens);
	NxActor* CreateStaticSphere(NxVec3 pos, float radius);
NxActor* CreateBox(NxVec3 pos, NxVec3 dim, float dens);
	NxActor* CreateStaticBox(NxVec3 pos, NxVec3 dim);
NxActor* CreateCapsule(const NxVec3& pos, const NxReal height, const NxReal radius, const NxReal density);
NxActor* CreatePrimaryMultiShape();
NxActor* CreateConvexMesh(NxVec3 pos, NxVec3 siz, float dens);
NxActor* CreateConvexMesh2(NxVec3 pos, NxVec3 siz, float dens);
NxActor* CreateTriangleMesh(NxVec3 pos, NxVec3 siz, float dens);

NxPulleyJoint* pulleyJoint(NxActor *obj1, NxActor *obj2, NxVec3 pulley1, NxVec3 pulley2, NxReal dist, NxReal ratio);
NxRevoluteJoint* motor(NxActor *actor1, NxActor* actor2, NxReal maxVel, NxVec3 anchor, NxVec3 axis);

void RotateObject(NxActor* actor, NxVec3 eixo, NxReal angle);

//Algumas flags
bool isPaused = false;
bool drawConvex = true;

//Um vetor com as estruturas de mesh
std::vector <Mesh> meshes;


//Defini��o da c�mera sint�tica
void SetOrthoForFont()
{	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
	glScalef(1, -1, 1);
	glTranslatef(0, -WINDOW_HEIGHT, 0);
	glMatrixMode(GL_MODELVIEW);	
	glLoadIdentity();
}

//Resetando a c�mera
void ResetPerspectiveProjection() 
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}


//Para escrever texto na tela
void RenderSpacedBitmapString( int x, int y, int spacing, void *font, char *string) 
{
	char *c;
	int x1=x;
	for (c=string; *c != '\0'; c++) {
		glRasterPos2i(x1,y);
		glutBitmapCharacter(font, *c);
		x1 = x1 + glutBitmapWidth(font,*c) + spacing;
	}
}

//Para desenhar os eixos na tela
void DrawAxes()
{	 
	//To prevent the view from disturbed on repaint
	//this push matrix call stores the current matrix state
	//and restores it once we are done with the arrow rendering
	glPushMatrix();
		glColor3f(0,0,1);
		glPushMatrix();
			glTranslatef(0,0, 0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label			
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "Z");
		glPopMatrix();					
		glutSolidCone(0.0225,1.0, 4,1);

		glColor3f(1,0,0);
		glRotatef(90,0,1,0);	
		glPushMatrix();
			glTranslatef(0,0,0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "X");
		glPopMatrix();					
		glutSolidCone(0.0225,1, 4,1);

		glColor3f(0,1,0);
		glRotatef(90,-1,0,0);	
		glPushMatrix();
			glTranslatef(0,0, 0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "Y");
		glPopMatrix();					
		glutSolidCone(0.0225,1, 4,1);	
	glPopMatrix();
}

//Para desenhar aquela grid em cima do plano de ch�o
void DrawGrid(int GRID_SIZE)
{
	glBegin(GL_LINES);
	glColor3f(0.75f, 0.75f, 0.75f);
	for(int i=-GRID_SIZE;i<=GRID_SIZE;i++)
	{
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);

		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}

//Para desenhar os modelos geom�tricos das meshes
void RenderMesh(Mesh m){ 

	float glmat[16];

	glEnable(GL_TEXTURE_2D);
   
	if (!m.actors.size() ) {
		glPushMatrix();
		m.actor->getGlobalPose().getColumnMajor44(glmat);
		glMultMatrixf(glmat);
	
	    // Para visualizar em 'wireframe', descomente esta linha
		//SetaModoDesenho('w');
		//Rotina da bibutil para desenhar objetos *OBJ
		DesenhaObjeto(m.obj);
		//E se estiver em wireframe, e quiseres desenhar o centro de massa, descomente essa
		//DrawPoint(m.actor->getCMassLocalPosition());
		glPopMatrix();
	}
	else {
		for(int i=0; i<m.actors.size(); i++){
			if(m.actors[i]){
				glPushMatrix();
				m.actors[i]->getGlobalPose().getColumnMajor44(glmat);
				glMultMatrixf(glmat);
	
				// Para visualizar em 'wireframe', descomente esta linha
				//SetaModoDesenho('w');
				DesenhaMesh(m.obj->meshes[i]);
				glPopMatrix();
			}
		}
	}
	
	glDisable(GL_TEXTURE_2D);
	
}

//Para chamar o passo da simula��o da PhysX
void StepPhysX() 
{ 
	if (!isPaused) {
		gScene->simulate(myTimestep);        
		gScene->flushStream();        
		//...perform useful work here using previous frame's state data        
		while(!gScene->fetchResults(NX_RIGID_BODY_FINISHED, false) )     
		{
			// do something useful        
		}
	}	
} 


void domino()
{
	//Sequ�ncia de domin�s
	float density = 0.2;
	for (int i = 0; i < 20; i++)
	{
		NxActor* box = CreateBox(NxVec3(i * 1.2 - 5,1,0), NxVec3 (0.1,1,0.5), density);
		gSelectedActor = box; //seleciona a caixa
		
		density += 0.1;
	}

	NxActor* rot_box = CreateBox(NxVec3(19.5,1, 0.5), NxVec3 (0.1,1,0.5), density);
	RotateObject(rot_box, NxVec3(0,1,0), -45);
	
	for (int i = 1; i < 5; i++)
	{
		NxActor* box = CreateBox(NxVec3(20,    1.5,      i * 1.2 + 0.5), NxVec3 (0.1,1.5,0.5), density);
		RotateObject(box, NxVec3(0,1,0), 90);

		density += 0.2;
	}

	NxActor* rot_box2 = CreateBox(NxVec3(19.5,1.5, 6.5), NxVec3 (0.1,1.5,0.5), density);
	RotateObject(rot_box2, NxVec3(0,1,0), 45);

	for (int i = 19; i > 0; i--)
	{
		NxActor* box = CreateBox(NxVec3(i * 1.2 - 5,2,8), NxVec3 (0.1,2,0.5), density);
		gSelectedActor = box; //seleciona a caixa 
		
		density += 1;
	}
}

void wall()
{
	CreateStaticBox(NxVec3(5,2,1), NxVec3(13,2,0.4));
	CreateStaticBox(NxVec3(5,1,7), NxVec3(11.5,0.8,0.4));
	CreateStaticBox(NxVec3(-8.5,2,4), NxVec3(0.2,2,3));


	RotateObject(CreateStaticBox(NxVec3(-7,1,4), NxVec3(0.2,2,3))
				 ,NxVec3(0,0,1), 45);

	RotateObject(CreateStaticBox(NxVec3(23,-0.15,4), NxVec3(1,0.5,5))
				 ,NxVec3(0,0,1), 10);

	CreateStaticBox(NxVec3(26,3,-1), NxVec3(5,3,0.2));
	CreateStaticBox(NxVec3(26,3,9), NxVec3(5,3,0.2));
	CreateStaticBox(NxVec3(31,3,4), NxVec3(0.2,8,5));

	CreateStaticBox(NxVec3(26,6,4), NxVec3(5,0.2,5));
	/*CreateStaticBox(NxVec3(28,1,1), NxVec3(3,0.5,2));
	CreateStaticBox(NxVec3(28,1,7), NxVec3(3,0.5,2));
	CreateStaticBox(NxVec3(30,1,4), NxVec3(1,0.5,2));
	CreateStaticBox(NxVec3(26,1,4), NxVec3(1,0.5,2));*/
}

void elevator()
{
	//NxActor* ball = CreateSphere2(NxVec3(-6,3,8), 1.0f, 45);
	

	NxActor* plat1 = CreateBox(NxVec3(-6,3.5, 8), NxVec3 (0.4,0.4,0.4), 80);
	NxActor* plat2 = CreateBox(NxVec3(-10,15, 8), NxVec3 (1,1.5,1), 80);

	pulleyJoint(plat1,plat2, NxVec3(-6,30,8), NxVec3(-10,30,8), 38, 1);

	//ball->putToSleep();
	plat1->putToSleep();
	plat2->putToSleep();
}

void gangorra()
{
	CreateBox(NxVec3(-10,1, 4), NxVec3 (1,0.1,4.5), 10);
	//CreateConvexMesh(NxVec3(-6,1,8), NxVec3(2,1,2), 30);
	CreateStaticSphere(NxVec3(-10,0.5, 4), 1.5);

	CreateBox(NxVec3(-10,3, 0.5), NxVec3 (0.5,0.5,0.5), 50);
}

void gangorra2()
{
	float d = 33;
	CreateBox(NxVec3(-9.5,1, 4 + d), NxVec3 (1,0.1,4.5), 10);
	//CreateConvexMesh(NxVec3(-6,1,8), NxVec3(2,1,2), 30);
	CreateStaticSphere(NxVec3(-9.5,0.5, 4 + d ), 1.5);

	CreateBox(NxVec3(-9.5,3, 7 +d ), NxVec3 (0.5,0.5,0.5), 5);
}


void ramp()
{
	NxReal x = -9.5;
	NxReal y = 22;
	NxReal z = 32;
	NxActor* box = CreateStaticBox(NxVec3(x,y,z), NxVec3 (2,0.5,8));
	NxActor* boxa = CreateStaticBox(NxVec3(x-2,y, z ), NxVec3 (0.2,2,8));
	NxActor* boxb = CreateStaticBox(NxVec3(x+2,y,z ), NxVec3 (0.2,2,8));
	RotateObject(box, NxVec3(1,0,0), -50);
	RotateObject(boxa, NxVec3(1,0,0), -50);
	RotateObject(boxb, NxVec3(1,0,0), -50);
	box->raiseBodyFlag(NX_BF_KINEMATIC);
	boxa->raiseBodyFlag(NX_BF_KINEMATIC);
	boxb->raiseBodyFlag(NX_BF_KINEMATIC);

	////////////////////////////////////////

	
}

void ramp2()
{
	NxReal x = -9.5;
	NxReal y = 12;
	NxReal z = 23;
	NxActor* box = CreateStaticBox(NxVec3(x,y,z), NxVec3 (1,0.5,5));
	NxActor* boxa = CreateStaticBox(NxVec3(x-1,y, z ), NxVec3 (0.2,2,5));
	NxActor* boxb = CreateStaticBox(NxVec3(x+1,y,z ), NxVec3 (0.2,2,5));
	RotateObject(box, NxVec3(1,0,0), -40);
	RotateObject(boxa, NxVec3(1,0,0), -40);
	RotateObject(boxb, NxVec3(1,0,0), -40);
	box->raiseBodyFlag(NX_BF_KINEMATIC);
	boxa->raiseBodyFlag(NX_BF_KINEMATIC);
	boxb->raiseBodyFlag(NX_BF_KINEMATIC);

	NxActor* ball = CreateSphere2(NxVec3(x,y+4.5,z +2.5), 0.5f, 500);
	ball->putToSleep();
	gSelectedActor = ball;
}

void ramp3()
{
	NxReal x = -8;
	NxReal y = 6;
	NxReal z = 14;
	NxActor* box = CreateStaticBox(NxVec3(x,y,z), NxVec3 (2,0.5,5));
	NxActor* boxa = CreateStaticBox(NxVec3(x-2,y+1, z ), NxVec3 (0.2,1,5));
	NxActor* boxb = CreateStaticBox(NxVec3(x+2,y+1,z ), NxVec3 (0.2,1,5));
	//RotateObject(box, NxVec3(1,0,0), -70);
	RotateObject(box, NxVec3(1,1,0), -20);
	RotateObject(boxa, NxVec3(1,1,0), -20);
	RotateObject(boxb, NxVec3(1,1,0), -20);
	box->raiseBodyFlag(NX_BF_KINEMATIC);
	boxa->raiseBodyFlag(NX_BF_KINEMATIC);
	boxb->raiseBodyFlag(NX_BF_KINEMATIC);

	//NxActor* ball = CreateSphere2(NxVec3(x,y+4.5,z +2.5), 0.8f, 500);
	//ball->putToSleep();
}

void motor()
{
	NxReal x = -1;
	NxReal y = 5;
	NxReal z = 4;

	NxReal speed = 5;
	NxReal boxdens = 1300;
	NxActor* Sphere = CreateSphere2(NxVec3(x,y,z), 0.5, 50);
	motor(Sphere, 
		  CreateBox(NxVec3(x,y-3,z), NxVec3 (0.1,2,2.5), boxdens), 
		  speed, NxVec3(x,y,z), NxVec3(0,0,-1));

	/*motor(Sphere, 
		  CreateBox(NxVec3(x,y+3,z), NxVec3 (0.1,2,2.5), boxdens), 
		  speed, NxVec3(x,y,z), NxVec3(0,0,-1));

	motor(Sphere, 
		  CreateBox(NxVec3(x+3,y,z), NxVec3 (2,0.1,2.5), boxdens), 
		  speed, NxVec3(x,y,z), NxVec3(0,0,-1));

	motor(Sphere, 
		  CreateBox(NxVec3(x-3,y,z), NxVec3 (2,0.1,2.5), boxdens), 
		  speed, NxVec3(x,y,z), NxVec3(0,0,-1));*/
	
}

void motor2()
{
	NxReal x = 5.5;
	NxReal y = 5;
	NxReal z = 4;
	
	NxReal speed = 5;
	NxReal boxdens = 1300;
	NxActor* Sphere = CreateSphere2(NxVec3(x,y,z), 0.5, 50);
	motor(Sphere, 
		  CreateBox(NxVec3(x,y-3,z), NxVec3 (0.1,2,2.5), boxdens), 
		  speed, NxVec3(x,y,z), NxVec3(0,0,-1));

	
}

void motor3()
{
	NxReal x = 11.5;
	NxReal y = 5;
	NxReal z = 4;
	
	NxReal speed = 5;
	NxReal boxdens = 1300;
	NxActor* Sphere = CreateSphere2(NxVec3(x,y,z), 0.5, 50);
	motor(Sphere, 
		  CreateBox(NxVec3(x,y-3,z), NxVec3 (0.1,2,2.5), boxdens), 
		  speed, NxVec3(x,y,z), NxVec3(0,0,-1));

	
}

void createMachine()
{
	domino();
	wall();
	elevator();
	gangorra();
	gangorra2();
	ramp();
	ramp2();
	ramp3();
	motor();
	motor2();
	motor3();
		
}

//Rotina que faz a inicializa��o da simula��o f�sica
void InitializePhysX() {
	
	//Define a inst�ncia da SDK
	gPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
	if(gPhysicsSDK == NULL) {
		cerr<<"Error creating PhysX device."<<endl;
		cerr<<"Exiting..."<<endl;
		exit(1);
	}

	//Cria a cena
	NxSceneDesc sceneDesc;
	sceneDesc.gravity.set(0.0f, -9.8f, 0.0f);
	sceneDesc.simType = NX_SIMULATION_SW;
	gScene = gPhysicsSDK->createScene(sceneDesc);

	//Seta par�metros da simula��o: timestep, max_iterations e m�todo de integra��o
	gScene->setTiming(myTimestep / 4.0f, 4, NX_TIMESTEP_FIXED);

	//Define o material padr�o
	NxMaterial* defaultMaterial = gScene->getMaterialFromIndex(0);
	defaultMaterial->setRestitution(0.5);
	defaultMaterial->setStaticFriction(0.5);
	defaultMaterial->setDynamicFriction(0.5);

	//Criar material madeira...

	//Cria o plano de ch�o
	NxPlaneShapeDesc planeDesc; //Descritor de forma de plano
	NxActorDesc actorDesc; //Descritor de corpo r�gido: ator
	actorDesc.shapes.pushBack(&planeDesc); //adiciona forma ao ator
	gScene->createActor(actorDesc); //cria o ator

	createMachine();

	////////////////////////////////////////////////////////////////////////////////////////


	//CreateConvexMesh(NxVec3(-6,1,8), NxVec3(2,1,2), 30);

	//CreateConvexMesh2(NxVec3(-6,1,8), NxVec3(2,1,2), 30);

	//float br = 2.0f;
	

	//Cria uma esfera
	//CreateSphere();

	//Cria uma c�psula
	//CreateCapsule();

	//Cria um ator com composi��o de formas
	//CreatePrimaryMultiShape();

	//CreateConvexMesh();

	//CreateTriangleMesh();
	
	//// Algumas dicas de como usar as malhas: descomente os trechos para ver o resultado	
	//Mesh m("./Data/lowerDuck.obj",NxVec3(0,5,0),20,true,gPhysicsSDK,gScene);
	//meshes.push_back(m);

	//Mesh m2("./Data/lowerDuck2.obj",NxVec3(5,5,0),20,false,gPhysicsSDK,gScene);
	//meshes.push_back(m2);

	//------------------------------------------
	 //Para testar as juntas REVOLUTE e FIXED
	//NxFixedJoint* fixedJoint;
	//NxRevoluteJoint* revJoint;
	//NxActor *box1 = CreateBox(NxVec3(0,5,0), NxVec3(0.5,0.5,1), 10);
	//box1->raiseBodyFlag(NX_BF_KINEMATIC);
	//NxActor *box2 = CreateBox(NxVec3(0,1,0), NxVec3(0.5,0.5,1), 10);
	//NxVec3 globalAnchor = NxVec3(0.5,5,0);
	//NxVec3 globalAxis = NxVec3(0,0,1);

	////fixedJoint = CreateFixedJoint(box1, box2, globalAnchor, globalAxis);
	//revJoint = CreateRevoluteJoint(box1, box2, globalAnchor, globalAxis);
	//gSelectedActor = box2;

	//--------------------------------------------
	// Para testar as juntas com MOLA 
	//NxActor *box1 = CreateBox(NxVec3(0,5,0), NxVec3(0.5,2,1), 10);
 //   box1->raiseBodyFlag(NX_BF_KINEMATIC);

 //   NxActor *box2 = CreateBox(NxVec3(0,1,0), NxVec3(0.5,2,1), 10);
 //   //box2->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);

 //   NxVec3 globalAnchor = NxVec3(0.5,5,-1);
 //   NxVec3 globalAxis = NxVec3(0,0,-1);
 //   revJoint = CreateRevoluteJointWithASpring(box1, box2, globalAnchor, globalAxis);
 //   gSelectedActor = box2;

	//----------------------------------------------
	// Para testar as juntas com um MOTOR
	/*NxActor *box1 = CreateBox(NxVec3(0,5,0), NxVec3(0.5,2,1), 10);
    box1->raiseBodyFlag(NX_BF_KINEMATIC);
    NxActor *box2 = CreateBox(NxVec3(0,1,0), NxVec3(0.5,2,1), 10);
    box2->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);

    box2->setAngularDamping(0);
    box2->setMaxAngularVelocity(20);

    NxVec3 globalAnchor = NxVec3(0.5,5,-1);
    NxVec3 globalAxis = NxVec3(0,0,-1);

    revJoint = CreateRevoluteJointWithAMotor(box1, box2, globalAnchor, globalAxis);
    gSelectedActor = box2;*/

	//-------------------------------------------------
	//Para testar as juntas SPHERICAL
	//NxActor *capsule1 = CreateCapsule(NxVec3(0,5,0), 3, 0.5, 10);
 //   capsule1->raiseBodyFlag(NX_BF_KINEMATIC);
 //   
	//NxActor *capsule2 = CreateCapsule(NxVec3(0,1,0), 3, 0.5, 10);
 //   capsule2->setLinearDamping(0.5);

 //   NxVec3 globalAnchor = NxVec3(0,5,0);
 //   NxVec3 globalAxis = NxVec3(0,1,0);
 //   
	//sphericalJoint = CreateSphericalJoint(capsule1, capsule2, globalAnchor, globalAxis);
	////sphericalJoint = CreateSphericalJointWithLimits(capsule1, capsule2, globalAnchor, globalAxis);
 //   gSelectedActor = capsule2;

	//-----------------------------------------------------
	//// Para testar as pulley joints
	//NxActor *capsule1 = CreateCapsule(NxVec3(-1,4,0), 1, 0.5, 10);
 //   NxActor *capsule2 = CreateCapsule(NxVec3(1,4,0), 1, 0.5, 10);
 //   // Motor specs
 //   gMotorDesc.maxForce = NX_MAX_REAL;		
 //   gMotorDesc.freeSpin = false;
 //   gMotorDesc.velTarget = 0;
 //   // Create pulley joint
 //   NxVec3 pulley1 = NxVec3(-1,8,0);
 //   NxVec3 pulley2 = NxVec3(1,8,0);
 //   NxVec3 globalAxis = NxVec3(0,-1,0);

 //   pulleyJoint = CreatePulleyJoint(capsule1, capsule2, pulley1, pulley2, globalAxis, 4, 1, gMotorDesc);
 //   gSelectedActor = capsule1;

}



NxPulleyJoint* pulleyJoint(NxActor *obj1, NxActor *obj2, NxVec3 pulley1, NxVec3 pulley2, NxReal dist, NxReal ratio)
{
	NxMotorDesc gMotorDesc;
	gMotorDesc.maxForce = NX_MAX_REAL;		
    gMotorDesc.freeSpin = false;
    gMotorDesc.velTarget = 0;

	NxVec3 globalAxis = NxVec3(0,-1,0);

	CreateStaticSphere(pulley1, 0.5f);
	CreateStaticSphere(pulley2, 0.5f);
	//ball->
	return CreatePulleyJoint(obj1, obj2, pulley1, pulley2, globalAxis, dist, ratio, gMotorDesc);
}

NxRevoluteJoint* motor(NxActor *actor1, NxActor* actor2, NxReal maxVel, NxVec3 anchor, NxVec3 axis)
{
	// Para testar as juntas com um MOTOR
	//NxActor *box1 = CreateBox(NxVec3(0,5,0), NxVec3(0.5,2,1), 10);
    actor1->raiseBodyFlag(NX_BF_KINEMATIC);
	//actor1->bod
    //NxActor *box2 = CreateBox(NxVec3(0,1,0), NxVec3(0.5,2,1), 10);
    actor2->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);

    actor2->setAngularDamping(0);
    actor2->setMaxAngularVelocity(maxVel);

    return CreateRevoluteJointWithAMotor(actor1, actor2, anchor, axis);
    //gSelectedActor = box2;
}



//Rotina que percorre os atores (corpos r�gidos) da cena e manda desenh�-los na tela
void RenderActors() 
{ 
    // Render all the actors in the scene 
    int nbActors = gScene->getNbActors(); 
    NxActor** actors = gScene->getActors(); 
    while (nbActors--) 
    { 
		glColor3f(1,1,1);	
        NxActor* actor = *actors++; 
		
		if (!actor->getShapes()[0]->isConvexMesh() && !actor->getShapes()[0]->isTriangleMesh())
			DrawActor(actor,gSelectedActor);
		else if (drawConvex)
			 DrawWireActor(actor);

		
		DrawActorShadow(actor);
        
		if (actor->getShapes()[0]->isPlane())
		{
			//Draw the grid and axes
			glDisable(GL_LIGHTING);
			DrawAxes();	
			DrawGrid(100);
			glEnable(GL_LIGHTING);
		}
		
    } 


} 

//Rotina para finalizar a simula��o 
void ShutdownPhysX() {
	gPhysicsSDK->releaseScene(*gScene);
	NxReleasePhysicsSDK(gPhysicsSDK);
}

//Rotina que verifica se o pr�ximo ator da lista pode ser selecionado ou n�o (se � din�mico, pode)
bool IsSelectable(NxActor* actor)
{
   NxShape*const* shapes = gSelectedActor->getShapes();
   NxU32 nShapes = gSelectedActor->getNbShapes();
   while (nShapes--)
   {
       if (shapes[nShapes]->getFlag(NX_TRIGGER_ENABLE)) 
       {           
           return false;
       }
   }

   if(!actor->isDynamic())
	   return false;

   if (actor == groundPlane)
       return false;

   return true;
}

//Rotina para selecionar pr�ximo ator
void SelectNextActor()
{
   NxU32 nbActors = gScene->getNbActors();
   NxActor** actors = gScene->getActors();
   for(NxU32 i = 0; i < nbActors; i++)
   {
       if (actors[i] == gSelectedActor)
       {
           NxU32 j = 1;
           gSelectedActor = actors[(i+j)%nbActors];
           while (!IsSelectable(gSelectedActor))
           {
               j++;
               gSelectedActor = actors[(i+j)%nbActors];
           }
           break;
       }
   }
}

//Rotina para a cria��o de uma esfera parametrizada (ainda usa material padr�o)
NxActor *CreateSphere(NxVec3 pos, float radius, float mass){
	//Rotina de cria��o aqui

	// Add a single-shape actor to the scene
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// The actor has one shape, a box, 1m on a side
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius		= radius;
	sphereDesc.localPose.t	= NxVec3(0, 0, 0); //posicao local, deixar em 0,0,0
	sphereDesc.materialIndex = 0; //default  //createMaterial(0.5,0.5,0.5);
	
	actorDesc.shapes.pushBack(&sphereDesc);

	actorDesc.body			= &bodyDesc;
	actorDesc.density		= NxComputeSphereDensity(sphereDesc.radius,mass);
	actorDesc.globalPose.t	= pos;	
	assert(actorDesc.isValid());  

	NxActor *pActor = gScene->createActor(actorDesc);	

    //pActor->setGlobalOrientationQuat(ori);

	return pActor;
}

//Rotina que converte graus para radianos
float GrausParaRadianos(float angulo)
{
	return angulo*NxPi/180;
}

//Rotina que converte radianos para graus
float RadianosParaGraus(float angle){
    return angle*180/NxPi;
}

NxVec3 CalculaComponentesDoVetor(float magnitude, float fi, float theta)
{
	NxVec3 vec;
	vec.x = magnitude * sin(fi) * cos(theta) ;
	vec.z = magnitude * sin(fi) * sin(theta) ;
	vec.y = magnitude * cos(fi);
	return vec;
}

//Rotina de callback de teclado
void KeyboardCallback(unsigned char key, int x, int y)
{

	switch (key)
	{
		case 'n':	
			{ 
				SelectNextActor();  
				break;
			}
		case ' ':
			{
				//Para lan�ar esferas: defini um vetor a partir de um ponto inicial e final no mundo
				NxVec3 posi(-8.0,0.0,0);
				NxVec3 posf(-7.5,0.5,0);
				NxVec3 vec(posf-posi); //dire��o do tiro, outra maneira seria determinando os angulos fi e theta e
				//NxVec3 vec(10,0);
				//calculando as componentes
				
				NxActor *sphere = CreateSphere(posi,0.45,2); // Rotina que cria esfera parametrizada
				NxVec3 forceVec;
				forceVec.multiply(1000,vec); //1000 � a magnitude, estou multiplicando pela dire��o

				//Adicionando a for�a ao centro de massa da esfera criada
				if(sphere)
					sphere->addForce(forceVec); 
				
				//Aqui � s� pra desenhar a seta na dire��o do lan�amento
				desenhaSeta = true;
				seta.ori= posi;
				seta.dir = vec;
				
				
				break;
				
			}
		case 'p':
			isPaused = !isPaused;
			break;
		case 27:
			{
				//ShutdownPhysX(); //t� dando erro ao liberar, ver isso mais tarde
				exit(0);
			}
	}
}

//Rotina que aplica uma for�a especificada em um ator
NxVec3 ApplyForceToActor(NxActor* actor, const NxVec3& forceDir, const NxReal forceStrength)
{
	NxVec3 forceVec = forceStrength*forceDir;
	
	actor->addForce(forceVec); //aplica a for�a no centro de massa 
	gForcePos = actor->getCMassGlobalPosition(); //para desenho depois
	
	return forceVec;
}

//Rotina de Callback de teclado especial
void SpecialCallback(int key, int x, int y)
{
	NxVec3 dir;
	bool ctrl = false;

	if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
           ctrl = true;
    }

	switch (key)
    {

		case GLUT_KEY_UP: 
			{ 
				if (!ctrl)
					dir = NxVec3(0,0,-1); //eixo Z
				else dir = NxVec3(0,1,0); //eixo Y
				isAppyingForce = true;
				break; 
			}
	    case GLUT_KEY_DOWN: 
			{ 
				if(!ctrl)
					dir = NxVec3(0,0,1); //eixo Z
				else
					dir = NxVec3(0,-1,0); //eixo Y
				isAppyingForce = true;
				break; 
			}
		case GLUT_KEY_LEFT: 
			{ 
				dir = NxVec3(-1,0,0);
				isAppyingForce = true;
				break; 
			}
	    case GLUT_KEY_RIGHT: 
			{ 
				dir = NxVec3(1,0,0);
				isAppyingForce = true;
				break; 
			}
		
	}

	if(isAppyingForce)
		gForceVec = ApplyForceToActor(gSelectedActor, dir,gForceStrength);

	glutPostRedisplay();

}

//Rotina de callback de teclado especial quando solta uma tecla
void SpecialUpCallback(int key, int x, int y)
{
	NxVec3 dir;
	switch (key)
    {

		case GLUT_KEY_UP: 
			{ 
				isAppyingForce = false;
				break; 
			}
	    case GLUT_KEY_DOWN: 
			{ 
				isAppyingForce = false;
				break; 
			}
		case GLUT_KEY_LEFT: 
			{ 
				isAppyingForce = false;
				break; 
			}
	    case GLUT_KEY_RIGHT: 
			{ 
				isAppyingForce = false;
				break; 
			}
		
	}

	glutPostRedisplay();
}

//Rotina para a inicializa��o dos par�metros da OpenGL
void InitGL() { 

	//Define o modo de opera��o da GLUT
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

	 glEnable(GL_DEPTH_TEST);
     glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float AmbientColor[]    = { 0.0f, 0.1f, 0.5f, 0.0f };         glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientColor);
    float DiffuseColor[]    = { 0.7f, 0.7f, 0.7f, 0.0f };         glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseColor);
    float SpecularColor[]   = { 0.5f, 0.5f, 0.5f, 0.0f };         glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularColor);

	glDisable(GL_LIGHTING);
}

//Rotina de callback para redimensionamento da tela
void OnReshape(int nw, int nh) {
	glViewport(0,0,nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)nw / (GLfloat)nh, 0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
}

//Rotina que faz o desenho de uma for�a aplicada
void DrawForce(NxActor* actor, NxVec3& forceVec, const NxVec3& color)
{
	// Draw only if the force is large enough
	NxReal force = forceVec.magnitude();
	if (force < 0.1)  return;

	forceVec = 3*forceVec/force;

	NxVec3 pos = gForcePos;
	DrawArrow(pos, pos + forceVec, color);
}


char buffer[MAX_PATH];
//Rotinca de callback de desenho
void OnRender() {

	// Clear buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Calculate fps
	totalFrames++;
	int current = glutGet(GLUT_ELAPSED_TIME);
	if((current-startTime)>1000)
	{		
		float elapsedTime = float(current-startTime);
		fps = ((totalFrames * 1000.0f)/ elapsedTime) ;
		startTime = current;
		totalFrames=0;
	}

	sprintf_s(buffer, "FPS: %3.2f",fps);

	//Update PhysX	
    if (gScene) 
    { 
        StepPhysX(); 
    } 

	glLoadIdentity();
	glTranslatef(0,0,dist);
	glRotatef(rX,1,0,0);
	glRotatef(rY,0,1,0);

	
	glEnable(GL_LIGHTING);
	
	//Para desenhar os atores da cena
	RenderActors();

	//Para renderizar a geometria associada as meshes
	for(int i=0;i<meshes.size();i++){
		RenderMesh(meshes[i]);
	}
	

	if(isAppyingForce)
		DrawForce(gSelectedActor, gForceVec, NxVec3(1,1,0));

	if (desenhaSeta)
	{
		DrawArrow(seta.ori, seta.ori+seta.dir*seta.length,NxVec3(1,0,0));
	}
	
	glDisable(GL_LIGHTING);
	SetOrthoForFont();		
	glColor3f(1,1,1);
	//Show the fps
	RenderSpacedBitmapString(20,20,0,GLUT_BITMAP_HELVETICA_12,buffer);

	//Se est� pausado, avisa
	if (isPaused)
		RenderSpacedBitmapString(20,40,0,GLUT_BITMAP_HELVETICA_18,"PAUSADO! PRESSIONE 'p' PARA CONTINUAR A SIMULACAO!");

	ResetPerspectiveProjection();

	glutSwapBuffers();
}

//Rotina chamada para finalizar a aplica��o
void OnShutdown() {
	ShutdownPhysX();
}


//Rotina de callback de mouse
void Mouse(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN) 
	{
		oldX = x; 
		oldY = y; 
	}	
	
	if(button == GLUT_MIDDLE_BUTTON)
		state = 0;
	else
		state = 1;
}

//Rotina de callback de movimenta��o do mouse
void Motion(int x, int y)
{
	if (state == 0)
		dist *= (1 + (y - oldY)/60.0f); 
	else
	{
		rY += (x - oldX)/5.0f; 
		rX += (y - oldY)/5.0f; 
	} 
	oldX = x; 
	oldY = y; 
	 
	glutPostRedisplay(); 
}

//Rotina de callback Idle
void OnIdle() {
	glutPostRedisplay();
}


//Rotina principal :)
void main(int argc, char** argv) {
	atexit(OnShutdown);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("Iniciando com a PhysX");

	glutDisplayFunc(OnRender);
	glutIdleFunc(OnIdle);
	glutReshapeFunc(OnReshape);

	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);

	glutKeyboardFunc(KeyboardCallback);
	glutSpecialFunc(SpecialCallback);
	glutSpecialUpFunc(SpecialUpCallback);

	InitGL();

	InitializePhysX();

	glutMainLoop();		
}


//Rotinas de Cria��o de Corpos R�gidos n�o parametrizados, para exemplo de aula

//Cria uma esfera
NxActor* CreateSphere2(NxVec3 pos, float radius, float dens)
{
	// Set the sphere starting height to 3.5m so box starts off falling onto the ground
	//NxReal sphereStartHeight = 3.5; 

	// Add a single-shape actor to the scene
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	// The actor has one shape, a sphere, 1m on radius
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius		= radius;;
	sphereDesc.localPose.t	= NxVec3(0, 0, 0);

	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= dens;
	actorDesc.globalPose.t	= pos;

	return gScene->createActor(actorDesc);
}

NxActor* CreateStaticSphere(NxVec3 pos, float radius)
{
	// Set the sphere starting height to 3.5m so box starts off falling onto the ground
	//NxReal sphereStartHeight = 3.5; 

	// Add a single-shape actor to the scene
	
	

	// The actor has one shape, a sphere, 1m on radius
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius		= radius;;
	sphereDesc.localPose.t	= NxVec3(0, 0, 0);

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.globalPose.t	= pos;

	//NxBodyDesc bodyDescriptor;
	actorDesc.body			= NULL;
	
	//actorDesc.raiseBodyFlag(NX_BF_KINEMATIC);

	return gScene->createActor(actorDesc);
}

NxActor* CreateCapsule(const NxVec3& pos, const NxReal height, const NxReal radius, const NxReal density)
{
	assert(0 != gScene);

	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.height = height;
	capsuleDesc.radius = radius;
	capsuleDesc.localPose.t = NxVec3(0, radius + 0.5f * height, 0);

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&capsuleDesc);
	actorDesc.globalPose.t = pos;

	NxBodyDesc bodyDesc;
	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = 0;
	}
	return gScene->createActor(actorDesc);	
}

NxActor* CreatePrimaryMultiShape()
{	
	// Create an actor which is composed of Box, Capsule and Sphere.
	NxActorDesc actorDesc;
	NxBodyDesc  bodyDesc;	
	
	//Box Shape
	NxBoxShapeDesc boxShape;
	boxShape.dimensions		= NxVec3(1.0f, 1.0f, 1.0f);
	boxShape.localPose.t	= NxVec3(1.5f, 0.0f, 0.0f);
	assert(boxShape.isValid());
	actorDesc.shapes.pushBack(&boxShape);

	//Capsule Shape
	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.radius		= 0.8f;
	capsuleDesc.height		= 1.0f;
	capsuleDesc.localPose.t = NxVec3(0.0f, 0.0f, 0.0f);
	//Rotate capsule 90 degree around z axis
	NxQuat quat(90.0f, NxVec3(0, 0, 1));
	NxMat33 m;
	m.id();
	m.fromQuat(quat);
	capsuleDesc.localPose.M = m;
	assert(capsuleDesc.isValid());
	actorDesc.shapes.pushBack(&capsuleDesc);

	//Sphere Shape
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius		= 1.0f;
	sphereDesc.localPose.t	= NxVec3(-1.5f, 0.0f, 0.0f);
	assert(sphereDesc.isValid());
	actorDesc.shapes.pushBack(&sphereDesc);

	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 1.0f;
	actorDesc.globalPose.t	= NxVec3(5.0f, 0.0f, 0.0f);
	assert(actorDesc.isValid());
	NxActor *pActor = gScene->createActor(actorDesc);
	assert(pActor);
return pActor;
}

NxActor* CreateStaticBox(NxVec3 pos, NxVec3 dim)
{
	
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	 
	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions.set(dim.x,dim.y,dim.z); 
	boxDesc.localPose.t = NxVec3(0, 0, 0); //posi��o local (em rela��o � caixa)
	//Adiciona a forma ao ator
	actorDesc.shapes.pushBack(&boxDesc);
	//O ator � um corpo r�gido, possui um corpo, uma densidade* e sua posi��o  no mundo
	actorDesc.body	= NULL;
	//actorDesc.density	= dens;
	NxVec3 posicao;
	
	actorDesc.globalPose.t	= pos;	
	assert(actorDesc.isValid());
 
	//Cria o ator em fun��o do seu descritor
	NxActor *pActor = gScene->createActor(actorDesc);	
 
	// Para iniciar a caixa rotacionada em 45 graus em Y
 //   NxVec3 v(0,1,0); //eixo de rota��o
 //   NxReal ang = 45; //�ngulo
 //   NxQuat q;
 //   q.fromAngleAxis(ang, v); //para transformar em um quaternion
	//pActor->setGlobalOrientationQuat(q);
 
	return pActor;
}

NxActor* CreateBox(NxVec3 pos, NxVec3 dim, float dens)
{
	
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	 
	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions.set(dim.x,dim.y,dim.z); 
	boxDesc.localPose.t = NxVec3(0, 0, 0); //posi��o local (em rela��o � caixa)
	//Adiciona a forma ao ator
	actorDesc.shapes.pushBack(&boxDesc);
	//O ator � um corpo r�gido, possui um corpo, uma densidade* e sua posi��o  no mundo
	actorDesc.body	= &bodyDesc;
	actorDesc.density	= dens;
	NxVec3 posicao;
	
	actorDesc.globalPose.t	= pos;	
	assert(actorDesc.isValid());
 
	//Cria o ator em fun��o do seu descritor
	NxActor *pActor = gScene->createActor(actorDesc);	
 
	// Para iniciar a caixa rotacionada em 45 graus em Y
 //   NxVec3 v(0,1,0); //eixo de rota��o
 //   NxReal ang = 45; //�ngulo
 //   NxQuat q;
 //   q.fromAngleAxis(ang, v); //para transformar em um quaternion
	//pActor->setGlobalOrientationQuat(q);
 
	return pActor;
}

void RotateObject(NxActor* actor, NxVec3 eixo, NxReal angle)
{
	NxQuat quaternion;
	quaternion.fromAngleAxis(angle,eixo);

	actor->setGlobalOrientationQuat(quaternion);
	
}


NxActor* CreateConvexMesh(NxVec3 pos, NxVec3 siz, float dens)
{
    NxActorDesc actorDesc;
    NxBodyDesc bodyDesc;
	
    NxVec3 boxDim(siz.x,siz.y,siz.z);
    // Pyramid
     NxVec3 verts[8] =	
		{	NxVec3(boxDim.x,      -boxDim.y,  -boxDim.z), 
		NxVec3(-boxDim.x,     -boxDim.y,  -boxDim.z), 
		NxVec3(-boxDim.x,     -boxDim.y,  boxDim.z),
		NxVec3(boxDim.x,      -boxDim.y,  boxDim.z), 
		NxVec3(boxDim.x*0.5,   boxDim.y,  -boxDim.z*0.5), 
		NxVec3(-boxDim.x*0.5,  boxDim.y,  -boxDim.z*0.5),
		NxVec3(-boxDim.x*0.5,  boxDim.y,  boxDim.z*0.5), 
		NxVec3(boxDim.x*0.5,   boxDim.y,  boxDim.z*0.5)
	};

	for (int i = 0; i < 8; i++)
	{
		verts[i].x += pos.x;
		verts[i].y += pos.y;
		verts[i].z += pos.z;
	}


	NxConvexMeshDesc *convexDesc = NULL;
	// Create descriptor for convex mesh
	if (!convexDesc)
	{
		convexDesc	= new NxConvexMeshDesc();
		assert(convexDesc);
	}
    	convexDesc->numVertices			= 8;
    	convexDesc->pointStrideBytes		= sizeof(NxVec3);
    	convexDesc->points				= verts;
	convexDesc->flags				= NX_CF_COMPUTE_CONVEX;

	NxConvexShapeDesc convexShapeDesc;
	convexShapeDesc.localPose.t		= NxVec3(0,boxDim.y,0);
	convexShapeDesc.userData		= convexDesc;
 
	
	// Two ways on cooking mesh: 1. Saved in memory, 2. Saved in file	
	NxInitCooking();
	// Cooking from memory
    	MemoryWriteBuffer buf;
    	bool status = NxCookConvexMesh(*convexDesc, buf);
	//
	// Please note about the created Convex Mesh, user needs to release it when no //one uses it to save memory. It can be detected
	// by API "meshData->getReferenceCount() == 0". And, the release API is //"gPhysicsSDK->releaseConvexMesh(*convexShapeDesc.meshData);"

	NxConvexMesh *pMesh = gPhysicsSDK->createConvexMesh(MemoryReadBuffer(buf.data));
	assert(pMesh);
    	convexShapeDesc.meshData	= pMesh;
	NxCloseCooking();

    if (pMesh)
    {
      // Save mesh in userData for drawing.
	pMesh->saveToDesc(*convexDesc);
	//
	NxActorDesc actorDesc;
	assert(convexShapeDesc.isValid());
      actorDesc.shapes.pushBack(&convexShapeDesc);
	actorDesc.body		= &bodyDesc;
	actorDesc.density		= 1.0f;
	   
      actorDesc.globalPose.t  = NxVec3(0.0f, 0.0f, 0.0f);
	assert(actorDesc.isValid());
	NxActor* actor = gScene->createActor(actorDesc);
	assert(actor);	
	return actor;
    }

    return NULL;
}

NxActor* CreateConvexMesh2(NxVec3 pos, NxVec3 siz, float dens)
{
    NxActorDesc actorDesc;
    NxBodyDesc bodyDesc;
	
    NxVec3 boxDim(siz.x,siz.y,siz.z);
    // Pyramid
     NxVec3 verts[8] =	
		{	NxVec3(boxDim.x,      -boxDim.y,  -boxDim.z), 
		NxVec3(-boxDim.x,     -boxDim.y,  -boxDim.z), 
		NxVec3(-boxDim.x,     -boxDim.y,  boxDim.z),
		NxVec3(boxDim.x,      -boxDim.y,  boxDim.z), 
		NxVec3(boxDim.x*0.5,   boxDim.y,  -boxDim.z*0.5), 
		NxVec3(boxDim.x*0.8,  boxDim.y,  -boxDim.z*0.5),
		NxVec3(boxDim.x*0.8,  boxDim.y,  boxDim.z*0.5), 
		NxVec3(boxDim.x*0.5,   boxDim.y,  boxDim.z*0.5)
	};

	for (int i = 0; i < 8; i++)
	{
		verts[i].x += pos.x;
		verts[i].y += pos.y;
		verts[i].z += pos.z;
	}


	NxConvexMeshDesc *convexDesc = NULL;
	// Create descriptor for convex mesh
	if (!convexDesc)
	{
		convexDesc	= new NxConvexMeshDesc();
		assert(convexDesc);
	}
    	convexDesc->numVertices			= 8;
    	convexDesc->pointStrideBytes		= sizeof(NxVec3);
    	convexDesc->points				= verts;
	convexDesc->flags				= NX_CF_COMPUTE_CONVEX;

	NxConvexShapeDesc convexShapeDesc;
	convexShapeDesc.localPose.t		= NxVec3(0,boxDim.y,0);
	convexShapeDesc.userData		= convexDesc;
 
	
	// Two ways on cooking mesh: 1. Saved in memory, 2. Saved in file	
	NxInitCooking();
	// Cooking from memory
    	MemoryWriteBuffer buf;
    	bool status = NxCookConvexMesh(*convexDesc, buf);
	//
	// Please note about the created Convex Mesh, user needs to release it when no //one uses it to save memory. It can be detected
	// by API "meshData->getReferenceCount() == 0". And, the release API is //"gPhysicsSDK->releaseConvexMesh(*convexShapeDesc.meshData);"

	NxConvexMesh *pMesh = gPhysicsSDK->createConvexMesh(MemoryReadBuffer(buf.data));
	assert(pMesh);
    	convexShapeDesc.meshData	= pMesh;
	NxCloseCooking();

    if (pMesh)
    {
      // Save mesh in userData for drawing.
	pMesh->saveToDesc(*convexDesc);
	//
	NxActorDesc actorDesc;
	assert(convexShapeDesc.isValid());
      actorDesc.shapes.pushBack(&convexShapeDesc);
	actorDesc.body		= &bodyDesc;
	actorDesc.density		= 1.0f;
	   
      actorDesc.globalPose.t  = NxVec3(0.0f, 0.0f, 0.0f);
	assert(actorDesc.isValid());
	NxActor* actor = gScene->createActor(actorDesc);
	assert(actor);	
	return actor;
    }

    return NULL;
}



NxActor* CreateTriangleMesh(NxVec3 pos, NxVec3 siz, float dens)
{
NxVec3 boxDim(1.0f, 1.0f, 1.0f);
	// Supply hull

	 NxVec3 verts[8] = 
		{	NxVec3(-boxDim.x, -boxDim.y, -boxDim.z), 
			NxVec3(boxDim.x,  -boxDim.y, -boxDim.z), 
			NxVec3(-boxDim.x, boxDim.y,  -boxDim.z), 
			NxVec3(boxDim.x,  boxDim.y,  -boxDim.z),
			NxVec3(-boxDim.x, -boxDim.y, boxDim.z), 
			NxVec3(boxDim.x,  -boxDim.y, boxDim.z), 
			NxVec3(-boxDim.x, boxDim.y,  boxDim.z), 
			NxVec3(boxDim.x,  boxDim.y,  boxDim.z),
		};


	for (int i = 0; i < 8; i++)
	{
		verts[i].x += pos.x;
		verts[i].y += pos.y;
		verts[i].z += pos.z;
	}

	// Triangles is 12*3
	 NxU32 indices[12*3] =
 		{	1,2,3,        
			0,2,1,  
			5,4,1,    
			1,4,0,    
			1,3,5,    
			3,7,5,    
			3,2,7,    
			2,6,7,    
			2,0,6,    
			4,6,0,
			7,4,5,
			7,6,4
		};

		// Create descriptor for triangle mesh
	 NxTriangleMeshDesc* triangleMeshDesc = NULL;
    	if (!triangleMeshDesc)
	{
		triangleMeshDesc	= new NxTriangleMeshDesc();
		assert(triangleMeshDesc);
	}
	triangleMeshDesc->numVertices		= 8;
	triangleMeshDesc->pointStrideBytes		= sizeof(NxVec3);
	triangleMeshDesc->points			= verts;
	triangleMeshDesc->numTriangles		= 12;
	triangleMeshDesc->flags			= 0;
	triangleMeshDesc->triangles			= indices;
	triangleMeshDesc->triangleStrideBytes	= 3 * sizeof(NxU32);
	
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
	tmsd.userData			= triangleMeshDesc;
	tmsd.localPose.t		= NxVec3(0, boxDim.y, 0);
	tmsd.meshPagingMode 	= NX_MESH_PAGING_AUTO;
	
	NxActorDesc actorDesc;
	NxBodyDesc  bodyDesc;
	
	assert(tmsd.isValid());
	actorDesc.shapes.pushBack(&tmsd);
	//Dynamic triangle mesh don't be supported anymore. So body = NULL
	actorDesc.body		= NULL;		
	actorDesc.globalPose.t	= NxVec3(3.0f, 0.0f, 0.0f);

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
