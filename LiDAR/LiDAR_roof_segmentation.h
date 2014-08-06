/* and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */

#ifndef LIDAR_ROOF_SEGMENTATION_H
#define LIDAR_ROOF_SEGMENTATION_H


#include "Gui.h"
#include <QtCore/QObject>
#include "ViewerShell.h"
//#include "AlgorithmShell.h"

//class Tutorial1 :public QObject, public AlgorithmShell // algorithm shell gives problem with teh gui...
class LiDAR_roof_segmentation :public QObject,public ViewerShell 
{
	Q_OBJECT
public:
   LiDAR_roof_segmentation();
   virtual ~LiDAR_roof_segmentation();

   bool getInputSpecification(PlugInArgList*& pInArgList);
   bool getOutputSpecification(PlugInArgList*& pOutArgList);
   bool execute(PlugInArgList* pInArgList, PlugInArgList* pOutArgList);
   
   /*bool serialize(SessionItemSerializer& serializer) const;
   bool deserialize(SessionItemDeserializer& deserializer);*/
   bool showGui();
  

 protected slots: 
   void LiDAR_roof_segmentation::dialogClosed();
 
protected:  
   QWidget* getWidget() const;

private:
  LiDAR_roof_segmentation(const LiDAR_roof_segmentation& rhs);
  LiDAR_roof_segmentation& operator=(const LiDAR_roof_segmentation& rhs);
  Gui* mpGui;
};

#endif
