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

console.log(wayPoints);

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

const Display = () => {
  const [robotStatus, setRobotStatus] = useState(null);
  const [robotOdometry, setRobotOdomety] = useState(null);
  const [currentPage, setCurrentPage] = useState("home");
  const [createdInfo, setCreatedInfo] = useState({
    executedCount: 0,
    goalX: 0,
    goalY: 0,
  })
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
    const feedbackCallback = (msg) => {
      setCreatedInfo({
        goalX: msg[0],
        goalY: msg[1],
        executedCount: msg[2]
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

    robotStatusTopic.subscribe(robotStatusCallback);
    odomTopic.subscribe(robotOdomPoseCallback);
    feedbackTopic.subscribe(feedbackCallback)

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
