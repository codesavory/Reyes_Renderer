#include "States.h"

ImageState::ImageState() {
	filename = nullptr;
}

FrameState::FrameState() {
	current_transformation <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
}
