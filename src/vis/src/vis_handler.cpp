#include "vis_handler.h"

VisHandler::VisHandler()
{
    camera_sub = n.subscribe("/robot/image", 1,
                            &VisHandler::camera_callback, this);
}

VisHandler::~VisHandler()
{
}
