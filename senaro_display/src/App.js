import { useEffect, useRef, useState } from "react";

import { Button } from "antd";
import ROSLIB, { Ros } from "roslib";

import DisplayHeader from "./features/display/DisplayHeader";
import DisplayMap from "./features/display/DisplayMap";
import DisplayStatus from "./features/display/DisplayStatus";
import DisplayCreateInfo from "./features/display/DisplayCreateInfo";
import {
  DisplayGlobalStyled,
  DisplayPageStyled,
} from "./styles/pageStyled/displayPageStyled";
import wayPoints from "./fake/waypoint.json";

const mmultiply = function(a,b) {
	return a.map(function(x,i) {
		return transpose(b).map(function(y,k) {
			return dotproduct(x, y)
		});
	});
}

const dotproduct = function(a,b) {
	return a.map(function(x,i) {
		return a[i] * b[i];
	}).reduce(function(m,n) { return m + n; });
}

const transpose = function(a) {
	return a[0].map(function(x,i) {
		return a.map(function(y,k) {
			return y[i];
		})
	});
}

const throttle = function (callback, waitMs) {
  let wait = false;
  return function (...args) {
    if (!wait) {
      wait = true;
      setTimeout(() => {
        wait = false;
        callback(...args);
      }, waitMs);
    }
  };
};

const qte = function(quat) {

  const q0 = quat[0];
  const q1 = quat[1];
  const q2 = quat[2];
  const q3 = quat[3];

  const Rx = Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - (2 * (q1 * q1 + q2 * q2)));
  const Ry = Math.asin(2 * (q0 * q2 - q3 * q1));
  const Rz = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - (2  * (q2 * q2 + q3 * q3)));

  const euler = [Rx, Ry, Rz];

  return(euler);
}

const Display = () => {
  const [robotStatus, setRobotStatus] = useState(null);
  const [robotOdometry, setRobotOdomety] = useState(null);
  const [map2OdomTf, setMap2OdomTf] = useState({
    x: 0,
    y: 0,
    z: 0,
    w: 1,
  })
  const [odom2baseTf, setOdom2baseTf] = useState({
    x: 0,
    y: 0,
    z: 0,
    w: 1,
  })
  const [currentPage, setCurrentPage] = useState("home");
  const [createdInfo, setCreatedInfo] = useState({
    executedCount: 0,
    goalX: 0,
    goalY: 0,
  })
  // const map2OdomMatrix = [
  //   [Math.cos(map2OdomTf.w), Math.sin(map2OdomTf.w), 0, map2OdomTf.x * Math.cos(map2OdomTf.w)],
  //   [Math.sin(map2OdomTf.w), Math.cos(map2OdomTf.w), 0, map2OdomTf.y * Math.sin(map2OdomTf.w)],
  //   [0, 0, 1, map2OdomTf.z],
  //   [0, 0, 0, 1],
  // ]
  // const odom2baseMatrix = [
  //   [Math.cos(odom2baseTf.w), Math.sin(odom2baseTf.w), 0, odom2baseTf.x * Math.cos(odom2baseTf.w)],
  //   [Math.sin(odom2baseTf.w), Math.cos(odom2baseTf.w), 0, odom2baseTf.y * Math.sin(odom2baseTf.w)],
  //   [0, 0, 1, odom2baseTf.z],
  //   [0, 0, 0, 1],
  // ]
  const map2OdomMatrix = [
    [Math.cos(map2OdomTf.w), Math.sin(map2OdomTf.w) * -1, 0, map2OdomTf.x],
    [Math.sin(map2OdomTf.w), Math.cos(map2OdomTf.w), 0, map2OdomTf.y],
    [0, 0, 1, map2OdomTf.z],
    [0, 0, 0, 1],
  ]
  const odom2baseMatrix = [
    [Math.cos(odom2baseTf.w), Math.sin(odom2baseTf.w) * -1, 0, odom2baseTf.x],
    [Math.sin(odom2baseTf.w), Math.cos(odom2baseTf.w), 0, odom2baseTf.y],
    [0, 0, 1, odom2baseTf.z],
    [0, 0, 0, 1],
  ]

  
  const map2BaseMatrix = mmultiply(map2OdomMatrix, odom2baseMatrix)
  // console.log(odom2baseMatrix)
  const mapTobaseTf = {
    x: map2BaseMatrix[0][3],
    y: map2BaseMatrix[1][3],
    z: map2BaseMatrix[2][3],
  }
  const connection = useRef(false);
  const mapCreateService = useRef(null);

  const openRosSocket = () => {
    connection.current = true;
    const robotStatusCallback = throttle((msg) => {
      setRobotStatus(msg);
    }, 1000);
    const robotOdomPoseCallback = throttle((msg) => {
      setRobotOdomety(msg);
    }, 60);
    const tfCallback = throttle((msg) => {
      const {x, y, z, w} = msg.transforms[0].transform.rotation
      const o = qte([w, x, y, z])
      if (msg.transforms[0].child_frame_id === 'odom') {
        console.log("odom", qte([w, x, y, z]))
        setMap2OdomTf({
          ...msg.transforms[0].transform.translation,
          w: o[2],
        })
      } else if (msg.transforms[0].child_frame_id === 'base_link') {
        setOdom2baseTf({
          ...msg.transforms[0].transform.translation,
          w: o[2],
        })
      }
    }, 60);
    const feedbackCallback = (msg) => {
      setCreatedInfo({
        goalX: msg.data[0],
        goalY: msg.data[1],
        executedCount: msg.data[2]
      })
    }
    const ros = new Ros({
      url: "ws://localhost:9090",
    });

    ros.on("connection", function () {
      console.log("Connection made!");
    });

    const robotStatusTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/tracer_status",
      messageType: "tracer_msgs/msgs/TracerStatus",
    });
    const odomTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/odom",
      messageType: "nav_msgs/msg/Odometry",
    });
    const feedbackTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/current_process",
      messageType: "std_msgs/Float32MultiArray",
    });
    const tfTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/tf",
      messageType: "tf2_msgs/msg/TFMessage",
    });

    robotStatusTopic.subscribe(robotStatusCallback);
    odomTopic.subscribe(robotOdomPoseCallback);
    feedbackTopic.subscribe(feedbackCallback)
    tfTopic.subscribe(tfCallback);

    mapCreateService.current = new ROSLIB.Service({
      ros: ros,
      name: "/create_map",
      serviceType: "senaro_interface/srv/CreateMap",
    });

    return () => {
      // robotStatusTopic.unsubscribe(robotStatusCallback);
      // odomTopic.unsubscribe(robotOdomPoseCallback);
    };
  };
  useEffect(() => {
    if (!connection.current) {
      return openRosSocket();
    }
  }, []);

  const sendCreateMapService = () => {
    if (mapCreateService.current) {
      console.log(mapCreateService.current)
      const flatWaypoints = Object.values(wayPoints.points).reduce((acc, val) => (
        [...acc, val[0], val[1]]
      ), [])
      const request = new ROSLIB.ServiceRequest({
        waypoints: flatWaypoints,
      });
      mapCreateService.current.callService(request, function (result) {
        console.log(result);
      });

      setCurrentPage("create");
    }
  };

  return (
    <DisplayPageStyled>
      {currentPage === "home" && (
        <>
          <DisplayGlobalStyled />
          <DisplayHeader />
          <DisplayMap
            origins={wayPoints.origins}
            markers={Object.values(wayPoints.points)}
            odometry={robotOdometry}
            mapToOdom={mapTobaseTf}
            goal={[createdInfo.goalX, createdInfo.goalY]}
          />
          <DisplayStatus robotStatus={robotStatus} />
          <div className="action">
            <Button
              block
              size="large"
              type="primary"
              onClick={() => {
                sendCreateMapService();
              }}
            >
              CREATE MAP
            </Button>
          </div>
        </>
      )}
      {currentPage === "create" && (
        <>
          <DisplayMap
            origins={wayPoints.origins}
            markers={Object.values(wayPoints.points)}
            odometry={robotOdometry}
            mapToOdom={map2OdomTf}
            goal={[createdInfo.goalX, createdInfo.goalY]}
          />
          <DisplayCreateInfo
            executedCount={createdInfo.executedCount}
            totalCount={Object.values(wayPoints.points).length}
            onEnd={() => {
              setCurrentPage("home");
            }}
          />
        </>
      )}
    </DisplayPageStyled>
  );
};

export default Display;
