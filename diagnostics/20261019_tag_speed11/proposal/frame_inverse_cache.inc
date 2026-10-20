	// Camera models are complete above and read-only for this computeTransform.
	// Compute each requested inverse once, using the original Transform::inverse.
	// Keep the cache frame-local and lazy: an unused/singular model is never inverted.
	std::vector<Transform> frameCameraInverses;
	std::vector<unsigned char> frameCameraInverseReady;
	const auto frameCameraInverse = [&](int cameraIndex) -> const Transform & {
		UASSERT(cameraIndex >= 0 && cameraIndex < (int)lastFrameModels.size());
		if(frameCameraInverseReady.empty())
		{
			frameCameraInverses.resize(lastFrameModels.size());
			frameCameraInverseReady.resize(lastFrameModels.size(), 0);
		}
		if(!frameCameraInverseReady[cameraIndex])
		{
			frameCameraInverses[cameraIndex] = lastFrameModels[cameraIndex].localTransform().inverse();
			frameCameraInverseReady[cameraIndex] = 1;
		}
		return frameCameraInverses[cameraIndex];
	};
