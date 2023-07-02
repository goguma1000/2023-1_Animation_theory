//
//  main.cpp
//  SpringMass
//
//  Created by Hyun Joon Shin on 2021/05/09.
//

#include <iostream>
#include <JGL/JGL_Window.hpp>
#include "AnimView.hpp"
#include <glm/gtx/quaternion.hpp>

using namespace glm;

struct Particle {
	vec3 x;
	vec3 v;
	vec3 f;
	float m;
	Particle( float mass, const vec3& position, const vec3& velocity=vec3(0) )
		: x( position ), v( velocity ), m( mass ) {}
	void clearForce() {
		f = vec3(0);
	}
	void add( const vec3& force ) {
		f+=force;
	}
	void update( float deltaT ) {
		x += v * deltaT;
		v += f / m * deltaT;
	}
	void draw() {
		drawSphere( x, 1 );
	}
};

struct Spring {
	Particle& a;
	Particle& b;
	float restLength;
	float k = 35.f;
	float kd = 0.1f;
	Spring( Particle& x, Particle& y ) : a(x), b(y), restLength( length(x.x-y.x)) {
	}
	void addForce() {
		//Damped Spring
		vec3 dx = a.x - b.x;
		vec3 dx_ = normalize(dx);
		vec3 f = -1 * (k * (length(dx) - restLength) + kd * dot(a.v-b.v,dx_)) * dx_;
		a.add(f);
		b.add(-f);
	}
	void draw() {
		drawCylinder( a.x, b.x, 0.4, glm::vec4(0,1,.4,1) );
	}
};

struct Plane {
	vec3 N;
	vec3 p;
	float alpha = 0.6; 
	float eps = 0.0001f;
	Plane( const vec3& position, const vec3& normal ): N(normal), p(position){}
	void draw() {
		drawQuad(p,N,{1000,1000},vec4(0,0,1,1));
	}
	void resolveCollision( Particle& particle ) {
		float d = dot(particle.x - p,N);
		if (d < eps) {
			float v = dot(N, particle.v);
			if (v < -eps) { 
				vec3 vn = v * N;
				vec3 vt = particle.v - vn;
				particle.v = vt - alpha * vn;
			}
			else if (v < eps) {
				vec3 vn = v * N;
				vec3 vt = particle.v - vn;
				particle.v = vt;
			}
			particle.x += -d * N; 
		}
	}
};

struct Sphere {
	float r;
	vec3 p;
	vec3 N;
	float alpha = 0.2;
	float mu = 0.5;
	float eps = 0.001f;
	Sphere(const float radius, const vec3 position) : r(radius), p(position){ }
	
	void draw() {
		drawSphere(p, r, { 1,0,1,1 });
	}

	void resolveCollision(Particle& particle, float dt) {
		const vec3 g(0, -980.f, 0);
		N = particle.x - p;
		float d = length(N) - r;
		vec3 N_ = normalize(N);
		if (d < eps) {
			float v = dot(N_, particle.v);
			if (v < -eps) {
				vec3 vn = v * N_;
				vec3 vt = particle.v - vn;
				vt = vt - alpha * vn;
				float Fn = dot(-particle.f, N_); 
				vt = vt - min(Fn * mu *dt / particle.m, length(vt)) * normalize(vt); 
				particle.v = vt;
			}
			else if (v < eps) { 
				vec3 vn = v * N_;
				vec3 vt = particle.v - vn;
				float Fn = dot(-particle.f, N_);
				vt =vt - min(Fn * mu * dt/ particle.m, length(vt)) * normalize(vt);
				particle.v = vt;
			}
			particle.x += -d * N_;
		}
	}
};
float randf() {
	return rand()/(float)RAND_MAX;
}
bool fix0 = true, fix1 = true;

void keyFunc(int key) {
	if( key == '1' )
		fix0=!fix0;
	if( key == '2' )
		fix1=!fix1;
}

const vec3 G ( 0, -980.f, 0 );
const float k_drag = 0.008f;
std::vector<Particle> particles;
std::vector<Spring> springs;
Plane flooring( {0,0,0}, {0,1,0} );
Sphere sphere(30, { 0,30,-5 });
const int count = 20;

void init() {
	particles.clear();
	springs.clear();
	for (int y= 0; y < count; y++) {
		for (int x = 0; x < count; x++) {
			particles.push_back(Particle(0.0008f, { x*2 -5.0f ,y * 2 + 62 ,randf() * 0.1f}));
		}
	}
	for (int y = 0; y < count; y++) {
		for (int x = 0; x < count-1; x++) {
			springs.push_back(Spring(particles[y * count + x], particles[y * count + (x + 1)]));
		}
	}
	for (int y = 0; y < count -1; y++) {
		for (int x = 0; x < count; x++) {
			springs.push_back(Spring(particles[y * count + x], particles[(y + 1) * count + x]));
		}
	}
	for (int y = 0; y < count - 1; y++) {
		for (int x = 0; x < count - 1; x++) {
			springs.push_back(Spring(particles[y * count + x], particles[(y + 1) * count + (x + 1)]));
		}
	}
	for (int y = 0; y < count - 1; y++) {
		for (int x = 0; x < count - 1; x++) {
			springs.push_back(Spring(particles[y * count + (x + 1)], particles[(y + 1) * count + x]));
		}
	}
}
void frame( float dt ) {
	const int steps =150;

	for (int i = 0; i<steps; i++)
	{
		vec3 p0 = particles[(count - 1) * count].x;
		vec3 p1 = particles[count * count -1].x;
		for (auto& p : particles) p.clearForce();
		for (auto& p : particles) p.add(p.m * G);
		for (auto& p : particles) p.add(-k_drag*p.v);
		for (auto& s : springs)	  s.addForce();
		for (auto& p : particles) p.update(dt / steps);
		for (auto& p : particles) flooring.resolveCollision(p);
		for (auto& p : particles) sphere.resolveCollision(p,dt/steps);
		if (fix0) {
			particles[(count - 1) * count].x = p0;
			particles[(count - 1) * count].v = {0,0,0};
		}

		if (fix1) {
			particles[count * count -1].x = p1;
			particles[count * count -1].v = { 0,0,0 };
		}
	}
	
}

void render() {
	for( auto& p : particles ) p.draw();
	for( auto& s : springs )   s.draw();
	flooring.draw();
	sphere.draw();

}

int main(int argc, const char * argv[]) {
	JGL::Window* window = new JGL::Window(800,600,"simulation");
	window->alignment(JGL::ALIGN_ALL);
	AnimView* animView = new AnimView(0,0,800,600);
	animView->renderFunction = render;
	animView->frameFunction = frame;
	animView->initFunction = init;
	animView->keyFunction = keyFunc;
	init();
	window->show();
	JGL::_JGL::run();
	return 0;
}
