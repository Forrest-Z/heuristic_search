//This file is copied from https://github.com/nathansttt/movingai/blob/master/utils/ScenarioLoader.h

/*
 * scenarioLoader.h
 * hog
 *
 * Created by Renee Jansen on 5/2/2006
 *
 */ 

#ifndef SCENARIOLOADER_H
#define SCENARIOLOADER_H

#include <vector>
#include <cstring>
#include <string>
#include <iostream>

namespace movingai_benchmark{
namespace loader{

using std::string;

static const int kNoScaling = -1;

/** 
 * Experiments stored by the ScenarioLoader class. 
 */
class ScenarioLoader;

class Experiment {
public:
	Experiment()
    :startx(0),starty(0),goalx(0),goaly(0),scaleX(kNoScaling),scaleY(kNoScaling),bucket(0),distance(0){}
	Experiment(int sx,int sy,int gx,int gy,int b, double d, string m)
    :startx(sx),starty(sy),goalx(gx),goaly(gy),scaleX(kNoScaling),scaleY(kNoScaling),bucket(b),distance(d),map(m){}
	Experiment(int sx,int sy,int gx,int gy,int sizeX, int sizeY,int b, double d, string m)
    :startx(sx),starty(sy),goalx(gx),goaly(gy),scaleX(sizeX),scaleY(sizeY),bucket(b),distance(d),map(m){}
	int GetStartX() const {return startx;}
	int GetStartY() const {return starty;}
	int GetGoalX() const {return goalx;}
	int GetGoalY() const {return goaly;}
	int GetBucket() const {return bucket;}
	double GetDistance() const {return distance;}
	void GetMapName(char* mymap) const {strcpy(mymap,map.c_str());}
	const char *GetMapName() const { return map.c_str(); }
	int GetXScale() const {return scaleX;}
	int GetYScale() const {return scaleY;}
	
        void SetMap(string map){this->map = map;}
        void SetStartX(int x){startx = x;}
        void SetStartY(int y){starty = y;}
        void SetGoalX(int x){goalx = x;}
        void SetGoalY(int y){goaly = y;}
protected:
	friend class ScenarioLoader;
	int startx, starty, goalx, goaly;
	int scaleX;
	int scaleY;
	int bucket;
	double distance;
	string map;
};

std::ostream &operator<<(std::ostream &stream, Experiment const& experiment);

/** A class which loads and stores scenarios from files.  
 * Versions currently handled: 0.0 and 1.0 (includes scale). 
 */

class ScenarioLoader{
public:
	ScenarioLoader() { scenName[0] = 0; }
	ScenarioLoader(const char *);
	void Save(const char *);
	int GetNumExperiments(){return experiments.size();}
	const char *GetScenarioName() { return scenName; }
	Experiment GetNthExperiment(int which)
	{return experiments[which];}
	void AddExperiment(Experiment which);
        
        std::vector<Experiment> &GetExperiments(){return experiments;}
private:
	char scenName[1024];
	std::vector<Experiment> experiments;
};

}//namespace loader
}//namespace movingai_benchmark
#endif
