#ifndef __BVH_HPP__
#define __BVH_HPP__

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <glm/gtx/quaternion.hpp>
#include "GLTools.hpp"

const float OFFSET_SCALE = 5.f;
const float RADIAN(3.141592 / 180.f);

inline glm::vec3 rotate( const glm::quat& q, const glm::vec3& v ) {
	glm::quat tmp = q*glm::quat(0,v)*inverse(q);
	return glm::vec3(tmp.x,tmp.y,tmp.z);
}

struct Link {
	enum class CHANNEL_TYPE {
		X_POSITION,
		Y_POSITION,
		Z_POSITION,
		X_ROTATION,
		Y_ROTATION,
		Z_ROTATION,
	};

	glm::vec3 offset;
	std::string name;
	std::vector<CHANNEL_TYPE> channelTypes;
	std::vector<Link*> children;

	glm::vec3 tr;
	glm::quat ro = glm::quat(1,0,0,0);

	Link* parent = nullptr;
	~Link() {
		for( int i=0; i<children.size(); i++ )
			delete children[i];
		children.clear();
	}
	void addChild(Link* l) {
		children.push_back(l);
	}
	void print(std::ostream& os, int t) {
		for( int i=0; i<t; i++ ) os<<" ";
		os<<name<<std::endl;
		for( auto child : children )
			child->print(os, t+1);
	}
	static Link* readJoint(std::istream& is, Link* parent) {
		Link* link = new Link();
		std::string tmp;
		int nChannels;
		is >> link->name; 
		is >> tmp; // {
		is >> tmp; // OFFSET
		is >> link->offset.x >> link->offset.y >> link->offset.z;
		link->offset *= OFFSET_SCALE;
		is >> tmp; // CHANNELS
		is >> nChannels;
		for(int i=0; i<nChannels; i++) {
			is >> tmp;
			if(tmp.compare("Xposition") == 0) link->channelTypes.push_back(CHANNEL_TYPE::X_POSITION);
			else if(tmp.compare("Yposition") == 0) link->channelTypes.push_back(CHANNEL_TYPE::Y_POSITION);
			else if(tmp.compare("Zposition") == 0) link->channelTypes.push_back(CHANNEL_TYPE::Z_POSITION);
			else if(tmp.compare("Xrotation") == 0) link->channelTypes.push_back(CHANNEL_TYPE::X_ROTATION);
			else if(tmp.compare("Yrotation") == 0) link->channelTypes.push_back(CHANNEL_TYPE::Y_ROTATION);
			else if(tmp.compare("Zrotation") == 0) link->channelTypes.push_back(CHANNEL_TYPE::Z_ROTATION);
		}
		while( true ) {
			is >> tmp; // JOINT, End, or }
			if(tmp.compare("JOINT")==0) link->addChild(readJoint(is,link));
			else if(tmp.compare("End")==0) link->addChild(readEndSite(is,link));
			else if(tmp.compare("}") ==0 ) break;
		}
		return link;
	}
	static Link* readEndSite(std::istream& is, Link* parent) {
		Link* link = new Link();
		std::string tmp;
		is >> tmp; // Site
		is >> link->name;
		is >> tmp; // {
		is >> tmp; // OFFSET
		is >> link->offset.x >> link->offset.y >> link->offset.z;
		link->offset *= OFFSET_SCALE;
		is >> tmp; // }
		return link;
	}
	void draw(const glm::vec3& pp, const glm::quat& pq) {
		glm::quat q = pq*ro;
		glm::vec3 p = rotate( q, offset ) + pp +tr;
		glm::mat4 m = translate( p );
		drawCylinder(p,pp,1,glm::vec4(1,0.3,0,1));
		drawSphere(p,2,glm::vec4(1,1,0,1));
		for( auto child: children )
			child->draw(p, q);
	}
};


struct Bone {
	enum class CHANNEL_TYPE {
		X_POSITION,
		Y_POSITION,
		Z_POSITION,
		X_ROTATION,
		Y_ROTATION,
		Z_ROTATION,
	};

	glm::vec3 offset;
	std::string name;
	std::vector<CHANNEL_TYPE> channelTypes;
	glm::vec3 tr = glm::vec3(0,0,0);
	glm::quat ro = glm::quat(1,0,0,0);
	int parent = -1;
	int dataOffset = 0;
	glm::vec3 gp;
	glm::quat gq;
};

struct Body {
	std::vector<Bone> bones;
	int Nframe = 0;
	float framerate = 0.f;
	std::vector<std::vector<float>> ValuesPerFrame;

	void readEndSite(std::istream& is, int parent ) {
		bones.push_back(Bone());
		Bone& bone = bones.back();
		bone.parent = parent;
		std::string tmp;
		is >> tmp; // Site
		is >> bone.name; std::cout<<bone.name<<std::endl;
		if (bone.name.compare("{") != 0) {
			is >> tmp; // {
		}
		is >> tmp; // OFFSET
		is >> bone.offset.x >> bone.offset.y >> bone.offset.z;
		bone.offset *= OFFSET_SCALE;
		is >> tmp; // }
	}

	void readBVH( const std::string& fn ) {
		std::ifstream is(fn);
		std::stack<int> parent;
		std::string tmp;
		is >> tmp; // HIERARCHY
		int dataIndex = 0;
		while( bones.size()==0 || !parent.empty() ) {

			is >> tmp; // JOINT, End, or }

			if(tmp.compare("JOINT")==0 ||  tmp.compare("ROOT")==0 )	{

				bones.push_back(Bone());
				Bone& bone = bones.back();
				bone.parent = parent.empty()?-1:parent.top();
				bone.dataOffset = dataIndex;
				int nChannels;
				is >> bone.name; std::cout<<bone.name<<std::endl;
				is >> tmp; // {
				is >> tmp; // OFFSET
				is >> bone.offset.x >> bone.offset.y >> bone.offset.z;
				bone.offset *= OFFSET_SCALE;
				is >> tmp; // CHANNELS
				is >> nChannels;
				for(int i=0; i<nChannels; i++) {
					is >> tmp;
					if(tmp.compare("Xposition") == 0)      bone.channelTypes.push_back(Bone::CHANNEL_TYPE::X_POSITION);
					else if(tmp.compare("Yposition") == 0) bone.channelTypes.push_back(Bone::CHANNEL_TYPE::Y_POSITION);
					else if(tmp.compare("Zposition") == 0) bone.channelTypes.push_back(Bone::CHANNEL_TYPE::Z_POSITION);
					else if(tmp.compare("Xrotation") == 0) bone.channelTypes.push_back(Bone::CHANNEL_TYPE::X_ROTATION);
					else if(tmp.compare("Yrotation") == 0) bone.channelTypes.push_back(Bone::CHANNEL_TYPE::Y_ROTATION);
					else if(tmp.compare("Zrotation") == 0) bone.channelTypes.push_back(Bone::CHANNEL_TYPE::Z_ROTATION);
				}
				dataIndex+=nChannels;
				parent.push( bones.size()-1 );
			}
			else if(tmp.compare("End")==0) readEndSite(is,parent.top());
			else if(tmp.compare("}") ==0 )
				parent.pop();
		}
		is >> tmp; //MOTION
		is >> tmp; //Frames
		is >> Nframe; //Nframe value
		is >> tmp >> tmp;//Frame Time:
		is >> framerate; //Frame Time value
		ValuesPerFrame.resize(Nframe, std::vector<float>(dataIndex));
		std::cout << Nframe << " :"<<framerate << " :" << dataIndex << std::endl;
		for (int i = 0; i < ValuesPerFrame.size(); i++) {
			for (int j = 0; j < ValuesPerFrame[i].size(); j++) {
				is >> ValuesPerFrame[i][j];
			}
		} // read value
		is.close();
	}
	void update() {//kinematic function
		for( auto& b: bones ) {
			if( b.parent>=0 ) {
				b.gq = bones[b.parent].gq *b.ro;
				b.gp = rotate(bones[b.parent].gq,b.offset) + b.tr + bones[b.parent].gp;
			}
			else {
				b.gq = b.ro;
				b.gp = rotate(b.gq,b.offset) + b.tr;
			}
		}
	}
	void draw() {
		update();
		for( auto& b: bones ) {
			if( b.parent>=0 )
				drawCylinder(b.gp,bones[b.parent].gp,1,glm::vec4(1,0,0,1));
		}
	}
	int getNFrames() const {
		return Nframe;
	}
	float getFrameRate() const {
		return framerate;
	}

	void updateBone(int framecount){
		for (auto& b : bones) {
			glm::quat q1 = {1,0,0,0};
			for (int i = 0; i < b.channelTypes.size(); i++) {
				if (b.channelTypes[i] == Bone::CHANNEL_TYPE::X_POSITION) {
					b.tr.x = ValuesPerFrame[framecount][b.dataOffset + i]*OFFSET_SCALE;
				}
				else if (b.channelTypes[i] == Bone::CHANNEL_TYPE::Y_POSITION) {
					b.tr.y = ValuesPerFrame[framecount][b.dataOffset + i] * OFFSET_SCALE;
				}
				else if (b.channelTypes[i] == Bone::CHANNEL_TYPE::Z_POSITION) {
					b.tr.z = ValuesPerFrame[framecount][b.dataOffset + i] * OFFSET_SCALE;
				}
				else if (b.channelTypes[i] == Bone::CHANNEL_TYPE::X_ROTATION) {
					q1*=glm::exp(glm::quat(0, glm::vec3(ValuesPerFrame[framecount][b.dataOffset + i],0,0)*RADIAN / 2.f));
				}
				else if (b.channelTypes[i] == Bone::CHANNEL_TYPE::Y_ROTATION) {
					q1*=glm::exp(glm::quat(0, glm::vec3(0,ValuesPerFrame[framecount][b.dataOffset + i],0)*RADIAN / 2.f));
				}
				else if (b.channelTypes[i] == Bone::CHANNEL_TYPE::Z_ROTATION) {
					q1*=glm::exp(glm::quat(0, glm::vec3(0, 0, ValuesPerFrame[framecount][b.dataOffset + i]) *RADIAN/ 2.f));
				}
			}
			b.ro = q1;
		}
	}
};



#endif
