import {
  FlyControls,
  OrbitControls,
  PerspectiveCamera,
  useTexture,
} from "@react-three/drei";
import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import { ReactNode, useEffect } from "react";
import { useRef } from "react";

import clsx from "clsx";
import {
  Camera,
  Color,
  ImageLoader,
  Mesh,
  TextureLoader,
  Quaternion,
} from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
import { DisplayMapStyled } from "./styled";
import React from "react";

const DisplayMapThreeJS = ({ robotPosition, markers, origins, goal }) => {
  const itemsRef = useRef(null);
  const robotRef = useRef(null);
  const camera = useRef(null);
  const orbitControl = useRef(null);
  const test = useLoader(TextureLoader, "/one.png");
  const mapMarker = useLoader(GLTFLoader, "/scene.gltf");
  const robot = useLoader(GLTFLoader, "/robot.glb");
  const color = new Color(0xffffff);
  const imageSource = test.source.data;


  useEffect(() => {
    if (orbitControl.current) {
      console.log(orbitControl.current);
      orbitControl.current.setPolarAngle(2.6);
    }
  }, [orbitControl]);

  const imageCenterXY = [
    (imageSource.width * 0.05) / 2,
    (imageSource.height * 0.05) / 2,
  ];
  const deltaCenterXY = [
    imageCenterXY[0] + origins[0],
    imageCenterXY[1] + origins[1],
  ];

  useEffect(() => {
    if (robotPosition) {
      camera.current.position.x = robotPosition.x * -1 + deltaCenterXY[0];
      camera.current.position.y = robotPosition.y * -1 + deltaCenterXY[1];
    }
  }, [robotPosition])

  useFrame(() => {
    if (itemsRef.current) {
      itemsRef.current.rotation.y += 0.1
      itemsRef.current.position.z = (Math.sin(itemsRef.current.rotation.y) + 5) * 0.2;
    }
  })

  return (
    <>
      <ambientLight intensity={1.4} />
      <directionalLight />
      <PerspectiveCamera
        ref={camera}
        args={[75, window.innerWidth / window.innerHeight, 1, 1100]}
      >
        <OrbitControls
          rotateSpeed={0.5}
          enableDamping={false}
          reverseOrbit={true}
          enablePan={false}
          enableRotate={false}
          maxAzimuthAngle={0.8}
          minAzimuthAngle={-0.8}
          maxPolarAngle={2.8}
          minPolarAngle={0.7}
          enableZoom={true}
          ref={orbitControl}
        />
        <mesh position={[0, 0, 0]} rotation={[0, 0, 0]}>
          <planeGeometry
            args={[imageSource.width * 0.05, imageSource.height * 0.05]}
          />
          <meshBasicMaterial map={test} color={color} />
        </mesh>
        <primitive
            ref={robotRef}
            object={robot.scene.clone()}
            position={[
              robotPosition.x - deltaCenterXY[0],
              robotPosition.y - deltaCenterXY[1],
              0.15,
            ]}
            rotation={[Math.PI / 2, 0, 0]}
            scale={0.02}
          />
        {markers.map((marker, index) => (
          <primitive
            key={index}
            ref={(el) => {
              if (marker[0] === goal[0] && marker[1] === goal[1]) {
                itemsRef.current = el
              }
            }}
            object={mapMarker.scene.clone()}
            position={[
              marker[0] - deltaCenterXY[0],
              marker[1] - deltaCenterXY[1],
              1,
            ]}
            rotation={[Math.PI / 2, 0, 0]}
            scale={0.3}
          />
        ))}
      </PerspectiveCamera>
    </>
  );
};

const DisplayMap = React.memo(({ className, odometry, mapToOdom, markers, origins, goal }) => {
  const robotPosition = odometry
    ? {
        ...odometry.pose.pose.position,
        ...odometry.pose.pose.orientation.w,
      }
    : { x: 0, y: 0, z: 0, w: 0 };

  robotPosition.x = mapToOdom.x
  robotPosition.y = mapToOdom.y
  // robotPosition.x += mapToOdom.x
  // robotPosition.y += mapToOdom.y

  robotPosition.z = mapToOdom.z
  const quaternion = new Quaternion(
    robotPosition.x,
    robotPosition.y,
    robotPosition.z,
    robotPosition.w
  );

  return (
    <DisplayMapStyled className={clsx("DisplayMap", className)}>
      <Canvas>
        <DisplayMapThreeJS
          robotPosition={quaternion}
          markers={markers}
          origins={origins}
          goal={goal}
        />
      </Canvas>
    </DisplayMapStyled>
  );
});

export default DisplayMap;
