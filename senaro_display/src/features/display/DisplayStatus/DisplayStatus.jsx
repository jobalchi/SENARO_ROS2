import { ReactNode } from "react";
import {
  CircularProgressbar,
  CircularProgressbarWithChildren,
} from "react-circular-progressbar";
import "react-circular-progressbar/dist/styles.css";

import clsx from "clsx";

import {
  DisplaySensorStyled,
  DisplayStatusCardStyled,
  DisplayStatusStyled,
} from "./styled";

const DisplayStatus = ({ className, robotStatus }) => {
  const batteryVoltage = robotStatus ? robotStatus.battery_voltage : 35;
  const robotMode = robotStatus ? robotStatus.control_mode : 1;
  return (
    <DisplayStatusStyled className={clsx("DisplayStatus", className)}>
      <DisplayStatusCardStyled className="battery">
        <div className="progressCircular">
          <CircularProgressbarWithChildren
            value={((batteryVoltage / 35) * 100).toFixed(0)}
          >
            <div>
              <i className="bx bxs-battery"></i>
            </div>
            <div className="percentage">
              {((batteryVoltage / 35) * 100).toFixed(0)}%
            </div>
          </CircularProgressbarWithChildren>
        </div>
      </DisplayStatusCardStyled>
      <DisplayStatusCardStyled className="controller">
        <i className="bx bx-joystick"></i>
        <div className="status">
          {robotMode === 1 && <>ROBOT MODE</>}
          {robotMode === 3 && <>CONTROLLER MODE</>}
        </div>
      </DisplayStatusCardStyled>
      <DisplayStatusCardStyled className="sensor">
        <DisplaySensorStyled>Lidar</DisplaySensorStyled>
        <DisplaySensorStyled>IMU</DisplaySensorStyled>
        <DisplaySensorStyled>Camera</DisplaySensorStyled>
        <DisplaySensorStyled>IR</DisplaySensorStyled>
      </DisplayStatusCardStyled>
    </DisplayStatusStyled>
  );
};

export default DisplayStatus;
