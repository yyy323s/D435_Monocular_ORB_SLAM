/**
 * File: FeatureVector.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#include "FeatureVector.h"
#include <map>
#include <vector>
#include <iostream>

namespace DBoW2 {

// ---------------------------------------------------------------------------

FeatureVector::FeatureVector(void)
{
}

// ---------------------------------------------------------------------------

FeatureVector::~FeatureVector(void)
{
}

// ---------------------------------------------------------------------------

void FeatureVector::addFeature(NodeId id, unsigned int i_feature)//这个函数用于把某个特征点编号 i_feature 归到词典树的节点 id 下。
{
  FeatureVector::iterator vit = this->lower_bound(id);//寻找第一个节点编号大于或等于 id 的位置
  
  if(vit != this->end() && vit->first == id)//如果该节点已经存在
  {
    vit->second.push_back(i_feature);
  }
  else
  {//FeatureVector 可以理解成：NodeId -> vector<特征点下标>，{12: [0, 4, 9],35: [1, 7]}
    vit = this->insert(vit, FeatureVector::value_type(id,
      std::vector<unsigned int>() ));//创建节点
    vit->second.push_back(i_feature);//
  }
}

// ---------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &out, 
  const FeatureVector &v)
{
  if(!v.empty())
  {
    FeatureVector::const_iterator vit = v.begin();
    
    const std::vector<unsigned int>* f = &vit->second;

    out << "<" << vit->first << ": [";
    if(!f->empty()) out << (*f)[0];
    for(unsigned int i = 1; i < f->size(); ++i)
    {
      out << ", " << (*f)[i];
    }
    out << "]>";
    
    for(++vit; vit != v.end(); ++vit)
    {
      f = &vit->second;
      
      out << ", <" << vit->first << ": [";
      if(!f->empty()) out << (*f)[0];
      for(unsigned int i = 1; i < f->size(); ++i)
      {
        out << ", " << (*f)[i];
      }
      out << "]>";
    }
  }
  
  return out;  
}

// ---------------------------------------------------------------------------

} // namespace DBoW2
