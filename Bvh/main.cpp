//
//  main.cpp
//  Quaternion
//
//  Created by Hyun Joon Shin on 2023/05/08.
//

#include <iostream>
#include <JGL/JGL_Window.hpp>
#include "AnimView.hpp"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include "bvh.hpp"

using namespace glm;
glm::quat q;
AnimView* animView = nullptr;
const float PI = 3.14159265358979;
glm::vec3 Euler = { PI / 4, PI / 3+PI*6, 0 };
int frameCount = 0;


void drawK( glm::vec3 pos, float sz=1, const glm::vec4& color=glm::vec4(1,0.4,0,1), const glm::mat4& mat = glm::mat4(1) ) {
	drawCylinder(pos+glm::vec3(-sz*2,-sz*3,0), pos+glm::vec3(-sz*2,sz*3,0), sz, color, mat);
	drawCylinder(pos+glm::vec3(-sz*2,-sz,0), pos+glm::vec3(sz*2,sz*3,0), sz, color, mat);
	drawCylinder(pos+glm::vec3(-sz,sz,0), pos+glm::vec3(sz*2,-sz*3,0), sz, color, mat);
}



Link* body;

void readBVH(const std::string & fn) {
	std::ifstream ifs(fn);
	std::string tmp;
	ifs >> tmp; // HIERARCHY
	ifs >> tmp; // Root
	body = Link::readJoint(ifs,nullptr);
	body->print(std::cout,0);
	ifs.close();
}
Body b;



void render() {
	drawQuad(glm::vec3(0), glm::vec3(0,1,0), glm::vec2(1000,1000), glm::vec4(0,0,1,1));
//	body->draw(vec3(0), quat(1,vec3(0)));
	b.draw();

}

void frame(float t) {
	frameCount = std::min(b.getNFrames()-1,(int)std::round(animView->progress()/b.getFrameRate()));
	b.updateBone(frameCount);
}

void init() {
	frameCount = 0;
}

int main(int argc, const char * argv[]) {
//	readBVH( "BackKickA.bvh" );
	b.readBVH( "SneezeA.bvh" );
//	body->tr = glm::vec3(0,30,0);
	b.bones[0].tr = glm::vec3(0,30,0);
	JGL::Window* window = new JGL::Window(640, 480, "simulation");
	window->alignment(JGL::ALIGN_ALL);
	animView = new AnimView(0, 0, 640, 480);
	animView->renderFunction = render;
	animView->frameFunction = frame;
	animView->initFunction = init;
	
	init();
	window->show();
	JGL::_JGL::run();
	return 0;
}
