import * as THREE from "three";

var image_buffer = new Map();

export default {
    template: ``,

    mounted() {

    },

    beforeDestroy() {

    },

    methods: {
        match(staticObjectID, referenceObjectID, dynamicObjectID, sceneID) {
            const parent = getElement(sceneID)
            if (parent === undefined) {
                return [new THREE.Vector3(), new THREE.Quaternion()];
            }
            var [staticObject, referenceObject, dynamicObject] = [parent.objects.get(staticObjectID), parent.objects.get(referenceObjectID), parent.objects.get(dynamicObjectID)]

            if (staticObject === undefined || referenceObject === undefined || dynamicObject === undefined) {
                return [new THREE.Vector3(), new THREE.Quaternion()];
            }

            var staticWorldPosition = new THREE.Vector3(), refernceWorldPosition = new THREE.Vector3();
            var staticWorldQuaternion = new THREE.Quaternion(), refernceWorldQuaternion = new THREE.Quaternion(), rotationDelta = new THREE.Quaternion();

            staticObject.getWorldPosition(staticWorldPosition);
            staticObject.getWorldQuaternion(staticWorldQuaternion);

            referenceObject.getWorldPosition(refernceWorldPosition);
            referenceObject.getWorldQuaternion(refernceWorldQuaternion);

            var inverse = refernceWorldQuaternion.clone().invert()
            var rotationDelta = staticWorldQuaternion.clone().premultiply(inverse)
            var localPositionDelta = referenceObject.worldToLocal(staticWorldPosition);
            dynamicObject.position.copy(localPositionDelta);
            dynamicObject.quaternion.copy(rotationDelta);
            return [dynamicObject.position, dynamicObject.quaternion.normalize()]
        },
        follow(staticObjectID, dynamicObjectID, sceneID) {
            const parent = getElement(sceneID);
            if (parent === undefined) {
                return [new THREE.Vector3(), new THREE.Quaternion()];
            }
            var staticObject = parent.objects.get(staticObjectID),
                dynamicObject = parent.objects.get(dynamicObjectID),
                staticWorldPosition = new THREE.Vector3(),
                staticWorldQuaternion = new THREE.Quaternion(),
                dynamicWorldQuaternion = new THREE.Quaternion();

            if (staticObject === undefined || dynamicObject === undefined) {
                return [staticWorldPosition, staticWorldQuaternion.normalize()]
            }

            staticObject.updateMatrixWorld(true);
            dynamicObject.updateMatrixWorld(true);

            staticObject.getWorldPosition(staticWorldPosition);
            staticObject.getWorldQuaternion(staticWorldQuaternion);
            dynamicObject.getWorldQuaternion(dynamicWorldQuaternion);

            dynamicObject.parent.worldToLocal(staticWorldPosition);
            staticWorldQuaternion.premultiply(dynamicWorldQuaternion.invert());

            dynamicObject.position.copy(staticWorldPosition);
            dynamicObject.quaternion.multiply(staticWorldQuaternion);
            return [dynamicObject.position, dynamicObject.quaternion.normalize()];
        },
        calcDistance(objectID1, objectID2, sceneID) {
            const parent = getElement(sceneID);
            if (parent === undefined) {
                return 0.0;
            }
            var object1 = parent.objects.get(objectID1),
                object2 = parent.objects.get(objectID2),
                worldPosition1 = new THREE.Vector3(),
                worldPosition2 = new THREE.Vector3();

            if (object1 === undefined || object2 === undefined) {
                return 0.0;
            }

            object1.getWorldPosition(worldPosition1);
            object2.getWorldPosition(worldPosition2);

            return worldPosition1.distanceToSquared(worldPosition2);
        },
        captureImage(sceneID) {
            const parent = getElement(sceneID);
            if (parent === undefined) {
                console.warn(`scene with id ${sceneID} not found.`);
                return '';
            }
            if (!image_buffer.has(sceneID)) {
                const originalRender = parent.renderer.render.bind(parent.renderer);

                parent.renderer.render = (...args) => {
                    originalRender(...args);
                    console.log(`rendering ${sceneID}`);
                    image_buffer.set(sceneID, parent.renderer.domElement.toDataURL("image/png"))
                };
            }
            return image_buffer.get(sceneID);
        },
        cameraHelper(sceneID) {
            let scene = getElement(sceneID);
            let camera = scene.camera;
            let helper = new THREE.CameraHelper(camera);
            scene.scene.add(helper);
        },
        attachCamera(sceneID, groupID) {
            let scene = getElement(sceneID);
            let group = scene?.objects?.get(groupID);
            let camera = scene?.camera;
            
            if (group === undefined || camera === undefined) {
                return false;
            }

            return group.attach(camera);
        }
    },

    props: {
    },
};
