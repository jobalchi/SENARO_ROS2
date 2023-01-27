import { ReactNode, useEffect, useState } from "react";

import clsx from "clsx";
import dayjs from "dayjs";

import { DisplayHeaderStyled } from "./styled";

const DisplayHeader = ({ className, children }) => {
  const [currentDatetime, setCurrentDatetime] = useState(dayjs());

  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentDatetime(dayjs());
    }, 1000);

    return () => {
      clearTimeout(timer);
    };
  }, []);
  return (
    <DisplayHeaderStyled className={clsx("DisplayHeader", className)}>
      <div className="logo">SENARO</div>
      <div className="status">
        {currentDatetime.format("MM월 DD일 HH:mm:ss")}
      </div>
    </DisplayHeaderStyled>
  );
};

export default DisplayHeader;
