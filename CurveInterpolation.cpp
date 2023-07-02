//
//  main.cpp
//  CurveInter
//
//  Created by Hyun Joon Shin on 2021/03/24.
//

#include <iostream>
#include <JGL/JGL.hpp>
#include <JGL/JGL_Window.hpp>
#include <JGL/JGL_Widget.hpp>
#include <JGL/JGL_Options.hpp>
#include <JGL/JGL_Toolbar.hpp>
#include <JGL/JGL_Aligner.hpp>
#include <Eigen/dense>


using glm::vec2;
using namespace std;


enum {
	LAGLANGIAN,
	LINEAR,
	BEZIER,
	HERMITE,
	CATMULL,
	OVERHAUSER,
	OVERHAUSER2,
	BSPLINE,
	NATURAL,
	NATURAL_CLOSED
};

enum {
	DRAW_LINES,
	DRAW_DOTS,
};

int curveType = LINEAR;
int drawType = DRAW_LINES;
bool closed = false;


std::vector<glm::vec2> srcPts;
std::vector<glm::vec2> samplePts;

template<typename T> T Bezier(const T& p0, const T& p1, const T& p2, const T& p3, float t) {
	float t1 = 1 - t;
	return t1 * t1 * t1 * p0 + 3 * t1 * t1 * t * p1 + 3 * t1 * t * t * p2 + t * t * t * p3;
}


vector<glm::vec2> evaluateCurve( int curveType, const std::vector<glm::vec2>& srcPts, bool closed, const vector<pair<int,float>>& samples ) {
	vector<vec2> ret;
	switch( curveType ) {
		case LAGLANGIAN: {
			for(auto [k, t] : samples) {
				vec2 pt(0);
				float T = k + t;
				for(auto i = 0; i < srcPts.size(); i++) {
					float L = 1;
					for(auto j = 0; j < srcPts.size(); j++) {
						if(j != i) {
							L /= (i - j);
							L *= T - j;
						}
					}
					pt += L * srcPts[i];
				}
				ret.push_back(pt);
			}
		} break;

		case BEZIER: {
			for(auto [k, t] : samples) {
				vec2 pt(0);
				if(k == 0) {
					pt = Bezier(srcPts[0],srcPts[1], srcPts[2], srcPts[3], t );
				}
				else {
					pt = srcPts[3];
				}
				ret.push_back(pt);
			}
		} break;
		case HERMITE: {
			for(auto [k, t] : samples) {
				vec2 v0 = vec2(120,0);
				vec2 v1 = vec2(90,0);
				vec2 p0 = srcPts[k];
				vec2 p3 = srcPts[k + 1];
				vec2 p1 = p0 + v0 / 3.f;
				vec2 p2 = p3 - v1 / 3.f;
				ret.push_back(Bezier(p0, p1, p2, p3, t));
			}
		} break;
		case CATMULL: {
			const glm::mat3 inverseMat = glm::inverse(glm::mat3({ {0,1,4}, {0,1,2}, {1,1,1} }));

			for(auto [k, t] : samples) {
				vec2 v0, v1;
				
				v0 = k > 0 ? (srcPts[k + 1] - srcPts[k - 1]) * 0.5f:vec2(0);
				v1 = k < srcPts.size() - 2 ? (srcPts[k + 2] - srcPts[k]) * 0.5f : vec2(0);
				
				if (k <= 0) {
					glm::vec3 x,b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k][c], srcPts[k + 1][c], srcPts[k + 2][c]);
						x = inverseMat * b;
						v0[c] = x[1];
					}

				}
				else if (k >= srcPts.size() - 2) {
					glm::vec3 x,b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k-1][c], srcPts[k][c], srcPts[k + 1][c]);
						x = inverseMat * b;
						v1[c] = 4*x[0]+x[1];
					}
				}

				vec2 p0 = srcPts[k];
				vec2 p3 = srcPts[k + 1];
				vec2 p1 = p0 + (v0 / 3.f);
				vec2 p2 = p3 - (v1 / 3.f);
				ret.push_back(Bezier(p0,p1,p2,p3,t));
			}
		} break;
		case OVERHAUSER: {
			const glm::mat3 m = inverse(glm::mat3(0, 1, 4, 0, 1, 2, 1, 1, 1));
			for(auto [k, t] : samples) {
				vec2 v0, v1;
				if (k <= 0) {
					glm::vec3 x, b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k][c], srcPts[k + 1][c], srcPts[k + 2][c]);
						x = m * b;
						v0[c] = x[1];
					}
				}
				else {
					glm::vec3 x, b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k-1][c], srcPts[k][c], srcPts[k+1][c]);
						x = m * b;
						v0[c] = 2*x[0] + x[1];
					}
				}

				if (k > srcPts.size() - 3) {
					glm::vec3 x, b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k - 1][c], srcPts[k][c], srcPts[k + 1][c]);
						x = m * b;
						v1[c] = 4 * x[0] + x[1];
					}
				}
				else {
					glm::vec3 x, b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k][c], srcPts[k + 1][c], srcPts[k + 2][c]);
						x = m * b;
						v1[c] = 2 * x[0] + x[1];
					}
				}
				vec2 p0 = srcPts[k];
				vec2 p3 = srcPts[k + 1];
				vec2 p1 = p0 + (v0 / 3.f);
				vec2 p2 = p3 - (v1 / 3.f);
				ret.push_back(Bezier(p0, p1, p2, p3, t));
			}
		} break;
		case OVERHAUSER2: {
			const glm::mat3 m = inverse(glm::mat3(0, 1, 4, 0, 1, 2, 1, 1, 1));
			vec2 pt1, pt2;
			for(auto [k, t] : samples) {
				if (k <= 0) {
					glm::vec3 x, b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k][c], srcPts[k + 1][c], srcPts[k + 2][c]);
						x = m * b;
						pt1[c] = x[0] * t * t + x[1] * t + x[2];
					}
				}
				else {
					glm::vec3 x, b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k - 1][c], srcPts[k][c], srcPts[k + 1][c]);
						x = m * b;
						pt1[c] = x[0] * (t + 1) * (t + 1) + x[1] * (t + 1) + x[2];
					}
				}

				if (k > srcPts.size() - 3) {
					glm::vec3 x, b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k - 1][c], srcPts[k][c], srcPts[k + 1][c]);
						x = m * b;
						pt2[c] = x[0] * (t + 1) * (t + 1) + x[1] * (t + 1) + x[2];
					}
				}
				else {
					glm::vec3 x, b;
					for (int c = 0; c < 2; c++) {
						b = glm::vec3(srcPts[k][c], srcPts[k + 1][c], srcPts[k + 2][c]);
						x = m * b;
						pt2[c] = x[0] * t * t + x[1] * t + x[2];
					}
				}

				ret.push_back(glm::mix(pt1, pt2, t));
			}

		}break;
		case BSPLINE: {
			for(auto [k, t] : samples) {
				vec2 pt;
				if(k < 1) pt = srcPts[1];
				else if(k > srcPts.size() - 3) pt = srcPts[srcPts.size() - 2];
				else {
					float w1 = 1 / 6.f * t * t * t;
					float w2 = 1 / 6.f * (-3 * t * t * t + 3 * t * t + 3*t + 1);
					float w3 = 1 / 6.f * (3 * t * t * t - 6 * t * t + 4);
					float w4 = 1 / 6.f * (1-t) * (1-t) * (1-t);
					pt = w4 * srcPts[k - 1] + w3 * srcPts[k] + w2 * srcPts[k + 1] + w1 * srcPts[k + 2];
				}

				ret.push_back(pt);
			}
		}break;

		case NATURAL: {
			int n = srcPts.size();
			Eigen::MatrixXf A = Eigen::MatrixXf::Zero(n, n);
			Eigen::VectorXf b(n);
			Eigen::VectorXf v(n);
			vec2 pt;

			A.row(0)[0] = 2;
			A.row(0)[1] = 1;
			for (int i = 0; i < n - 2; i++) {
				A.row(i + 1)[i] = 1;
				A.row(i + 1)[i + 1] = 4;
				A.row(i + 1)[i + 2] = 1;
			}
			A.row(n-1)[n - 2] = 1;
			A.row(n-1)[n - 1] = 2;

			b[0] = 3.f * (srcPts[1].x - srcPts[0].x);
			for (int i = 0; i < n - 2; i++) {
				b[i + 1] = 3.f * (srcPts[i + 2].x - srcPts[i].x);
			}
			b[n - 1] = 3.f * (srcPts[n - 1].x - srcPts[n - 2].x);

			Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);

			v[0] = 3.f * (srcPts[1].y - srcPts[0].y);
			for (int i = 0; i < n - 2; i++) {
				v[i + 1] = 3.f * (srcPts[i + 2].y - srcPts[i].y);
			}
			v[n - 1] = 3.f * (srcPts[n - 1].y - srcPts[n - 2].y);

			Eigen::VectorXf y = A.colPivHouseholderQr().solve(v);

			for(auto [k, t] : samples) {
				float a = srcPts[k].x;
				float b = x[k];
				float c = 3.f * (srcPts[k + 1].x - srcPts[k].x) - 2.f * x[k] - x[k+1];
				float d = 2.f * (srcPts[k].x - srcPts[k+1].x) + x[k] + x[k + 1];
				pt.x = a + b * t + c * t * t + d * t * t * t;
				
				a = srcPts[k].y;
				b = y[k];
				c = 3.f * (srcPts[k + 1].y - srcPts[k].y) - 2.f * y[k] - y[k + 1];
				d = 2.f * (srcPts[k].y - srcPts[k + 1].y) + y[k] + y[k + 1];
				pt.y = a + b * t + c * t * t + d * t * t * t;
				
				ret.push_back(pt);
			}

			/*int n = srcPts.size();
			Eigen::MatrixXf A = Eigen::MatrixXf::Zero(n, n);
			Eigen::VectorXf b(n);
			Eigen::VectorXf e1(n), e2(n), e3(n), e4(n);
			Eigen::VectorXf v1(n), v2(n), v3(n), v4(n);
			vec2 pt;

			A.row(0)[0] = 2;
			A.row(0)[1] = 1;
			for (int i = 0; i < n - 2; i++) {
				A.row(i + 1)[i] = 1;
				A.row(i + 1)[i + 1] = 4;
				A.row(i + 1)[i + 2] = 1;
			}
			A.row(n - 1)[n - 2] = 1;
			A.row(n - 1)[n - 1] = 2;

			b[0] = 3.f * (srcPts[1].x - srcPts[0].x);
			for (int i = 0; i < n - 2; i++) {
				b[i + 1] = 3.f * (srcPts[i + 2].x - srcPts[i].x);
			}
			b[n - 1] = 3.f * (srcPts[n - 1].x - srcPts[n - 2].x);

			Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);

			for (int i = 0; i < n - 1; i++) {
				e1[i] = srcPts[i].x;
				e2[i] = x[i];
				e3[i] = 3.f * (srcPts[i + 1].x - srcPts[i].x) - 2.f * x[i] - x[i + 1];
				e4[i] = 2.f * (srcPts[i].x - srcPts[i + 1].x) + x[i] + x[i + 1];
			}
			b[0] = 3.f * (srcPts[1].y - srcPts[0].y);
			for (int i = 0; i < n - 2; i++) {
				b[i + 1] = 3.f * (srcPts[i + 2].y - srcPts[i].y);
			}
			b[n - 1] = 3.f * (srcPts[n - 1].y - srcPts[n - 2].y);

			Eigen::VectorXf y = A.colPivHouseholderQr().solve(b);

			for (int i = 0; i < n - 1; i++) {
				v1[i] = srcPts[i].y;
				v2[i] = y[i];
				v3[i] = 3.f * (srcPts[i + 1].y - srcPts[i].y) - 2.f * y[i] - y[i + 1];
				v4[i] = 2.f * (srcPts[i].y - srcPts[i + 1].y) + y[i] + y[i + 1];
			}

			for (auto [k, t] : samples) {

				pt.x = e1[k] + e2[k] * t + e3[k] * t * t + e4[k] * t * t * t;
				pt.y = v1[k] + v2[k] * t + v3[k] * t * t + v4[k] * t * t * t;

				ret.push_back(pt);
			}*/
		} break;

		case NATURAL_CLOSED: {
			int n = srcPts.size();
			Eigen::MatrixXf A = Eigen::MatrixXf::Zero(n, n);
			Eigen::VectorXf b(n);
			Eigen::VectorXf e1(n), e2(n), e3(n), e4(n);
			Eigen::VectorXf v1(n), v2(n), v3(n), v4(n);
			vec2 pt;
			A.row(0)[0] = 4;
			A.row(0)[1] = 1;
			A.row(0)[n-1] = 1;

			for (int i = 1; i < n - 1; i++) {
				A.row(i)[i - 1] = 1;
				A.row(i)[i] = 4;
				A.row(i)[i+1] = 1;

			}

			A.row(n-1)[0] = 1;
			A.row(n - 1)[n - 2] = 1;
			A.row(n - 1)[n - 1] = 4;

			b[0] = 3.f * (srcPts[1].x - srcPts[n - 1].x);
			for (int i = 1; i < n - 1; i++) {
				b[i] = 3.f * (srcPts[i+1].x - srcPts[i - 1].x);
			}
			b[n-1] = 3.f * (srcPts[0].x - srcPts[n - 2].x);

			Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);
			for (int i = 0; i < n; i++) {
				if (i == n - 1) {
					e1[i] = srcPts[i].x;
					e2[i] = x[i];
					e3[i] = 3.f * (srcPts[0].x - srcPts[i].x) - 2.f * x[i] - x[0];
					e4[i] = 2.f * (srcPts[i].x - srcPts[0].x) + x[i] + x[0];
				}
				else {
					e1[i] = srcPts[i].x;
					e2[i] = x[i];
					e3[i] = 3.f * (srcPts[i + 1].x - srcPts[i].x) - 2.f * x[i] - x[i + 1];
					e4[i] = 2.f * (srcPts[i].x - srcPts[i + 1].x) + x[i] + x[i + 1];
				}
			}

			b[0] = 3.f * (srcPts[1].y - srcPts[n - 1].y);
			for (int i = 1; i < n - 1; i++) {
				b[i] = 3.f * (srcPts[i + 1].y - srcPts[i - 1].y);
			}
			b[n - 1] = 3.f * (srcPts[0].y - srcPts[n - 2].y);

			Eigen::VectorXf y = A.colPivHouseholderQr().solve(b);

			for (int i = 0; i < n; i++) {
				if (i == n - 1) {
					v1[i] = srcPts[i].y;
					v2[i] = y[i];
					v3[i] = 3.f * (srcPts[0].y - srcPts[i].y) - 2.f * y[i] - y[0];
					v4[i] = 2.f * (srcPts[i].y - srcPts[0].y) + y[i] + y[0];
				}
				else {
					v1[i] = srcPts[i].y;
					v2[i] = y[i];
					v3[i] = 3.f * (srcPts[i + 1].y - srcPts[i].y) - 2.f * y[i] - y[i + 1];
					v4[i] = 2.f * (srcPts[i].y - srcPts[i + 1].y) + y[i] + y[i + 1];
				}
			}

			for (auto [k, t] : samples) {
				pt.x = e1[k] + e2[k] * t + e3[k] * t * t + e4[k] * t * t * t;
				pt.y = v1[k] + v2[k] * t + v3[k] * t * t + v4[k] * t * t * t;

				ret.push_back(pt);
			}
			pt.x = e1[0];
			pt.y = v1[0];
			ret.push_back(pt);
		}break;
		case LINEAR :
		default: {
			for(auto [k, t] : samples) {
				vec2 pt = (srcPts[k + 1] - srcPts[k]) * t + srcPts[k];
				ret.push_back(pt);
			}
		}
	}
	return ret;
}

void updateCurve( int curveType, const std::vector<glm::vec2>& srcPts, bool closed ) {
	samplePts.clear();
	vector<pair<int,float>> sampleT;
	for(auto i = 0; i < srcPts.size() - 1; i++) {
		for(float t = 0; t < 1; t += 0.1) {
			sampleT.push_back({ i,t });
		}
	}
	if(closed) {
		for(float t = 0; t < 1; t += 0.1) {
			sampleT.push_back({ srcPts.size() - 1, t });
		}
	}
	else
		sampleT.push_back({ srcPts.size() - 2, 1 });
	samplePts = evaluateCurve( curveType, srcPts, closed, sampleT );
}


#include <JGL/nanovg/nanovg.h>


struct CurveWidget : JGL::Widget {
	CurveWidget(float x, float y, float w, float h, const std::string& title = "" )
	: JGL::Widget(x,y,w,h,title){}
	virtual void		drawBox(NVGcontext* vg, const glm::rect& r) override {
		nvgSave(vg);
		nvgBeginPath( vg );
		nvgRect( vg, r.x, r.y, r.w, r.h );
		nvgFillColor( vg, nvgRGBAf(0,0,0,1));
		nvgFill( vg );
		nvgRestore(vg);
	}
	
	virtual void drawContents(NVGcontext* vg, const glm::rect& r, int align ) override {
		nvgSave(vg);
		if( drawType == DRAW_LINES ) {
			nvgBeginPath( vg );
			nvgMoveTo( vg, samplePts[0].x, samplePts[0].y );
			for( auto i=1; i<samplePts.size(); i++ ) {
				nvgLineTo( vg, samplePts[i].x, samplePts[i].y );
			}
			nvgStrokeColor(vg, nvgRGBAf(0,.8f,1,1));
			nvgStrokeWidth(vg, 2);
			nvgStroke( vg );
		}
		else {
			nvgFillColor(vg, nvgRGBAf(0,1,.3f,1));
			nvgBeginPath( vg );
			for( auto i=0; i<samplePts.size(); i++ ) {
				nvgCircle(vg, samplePts[i].x, samplePts[i].y, 1);
			}
			nvgFill( vg );
		}

		for( auto i=0; i<srcPts.size(); i++ )
			if( i!= underPt ) {
				nvgBeginPath( vg );
				nvgCircle( vg, srcPts[i].x, srcPts[i].y, 5 );
				nvgFillColor( vg, nvgRGBAf(1,1,0,.8f));
				nvgFill( vg );
			}
		if( underPt>=0 ) {
			nvgBeginPath( vg );
			nvgCircle( vg, srcPts[underPt].x, srcPts[underPt].y, 5 );
			nvgFillColor( vg, nvgRGBAf(1,.1f,0,.8f));
			nvgFill( vg );
		}
		nvgRestore(vg);
	}
	virtual bool handle( int event ) override {
		glm::vec2 pt = JGL::_JGL::eventPt();
		switch( event ) {
			case JGL::EVENT_MOVE : {
				int oldPt = underPt;
				underPt = -1;
				for( auto i=0; i<srcPts.size(); i++ ) {
					if( length(pt-srcPts[i])<6 )
						underPt = i;
				}
				if( underPt!= oldPt ) {
					redraw();
				}
			}break;
			case JGL::EVENT_PUSH : {
				if( underPt>=0 )
					ptOffset = srcPts[underPt]-pt;
			}break;
			case JGL::EVENT_DRAG : {
				if( underPt>=0 ) {
					srcPts[underPt] = pt+ptOffset;
					updateCurve( curveType, srcPts, closed );
					redraw();
				}
			}break;
		}
		return true;
	}
	int underPt = -1;
	glm::vec2 ptOffset = glm::vec2(0);
};


using namespace JGL;
CurveWidget* curveWidget;


void curveTypeCallback(Widget* w, void* ud ) {
	curveType = ((Options*)w)->value();
	if (curveType == NATURAL_CLOSED) closed = true;
	else closed = false;
	updateCurve(curveType, srcPts, closed);
	curveWidget->redraw();
}

void drawTypeCallback(Widget* w, void* ud ) {
	drawType = ((Options*)w)->value();
	curveWidget->redraw();
}

int main(int argc, const char * argv[]) {
	Window* window = new Window(640,480,"Curves");
	window->alignment(ALIGN_ALL);
	
	Aligner* aligner = new Aligner(0,0,window->w(), window->h());
	aligner->type(Aligner::VERTICAL);
	aligner->alignment(ALIGN_ALL);
	
	Toolbar* toolbar = new Toolbar(0,0,window->w(), _size_toolbar_height() );
	Options* curveTypes = new Options(0,0,200,_size_button_height() );
	curveTypes->add("Lagrangian");
	curveTypes->add("Linear");
	curveTypes->add("Bezier");
	curveTypes->add("Hermite");
	curveTypes->add("Catmull");
	curveTypes->add("Overhauser");
	curveTypes->add("Overhauser2");
	curveTypes->add("Bspline");
	curveTypes->add("Natural Spline");
	curveTypes->add("Natural Closed");
	curveTypes->value(curveType);
	curveTypes->callback( curveTypeCallback );
	Options* drawType = new Options(0,0,200,_size_button_height() );
	drawType->add("Lines");
	drawType->add("Dots");
	drawType->value(::drawType);
	drawType->callback( drawTypeCallback );
	toolbar->end();
	
	curveWidget = new CurveWidget(0,0,window->w(), window->h()-toolbar->h());
	aligner->resizable( curveWidget );
	aligner->end();
	
	for( auto i=0; i<9; i++ ) {
		srcPts.push_back(glm::vec2((i*0.1+0.1)*curveWidget->w(),curveWidget->h()/2));
	}
	updateCurve(curveType,srcPts,closed);
	
	window->show();
	_JGL::run();
	
	return 0;
}
