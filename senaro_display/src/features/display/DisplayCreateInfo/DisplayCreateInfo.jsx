import ProgressBar from "@ramonak/react-progress-bar";
import { ReactNode, useState } from "react";
import "react-circular-progressbar/dist/styles.css";

import { Button } from "antd";
import clsx from "clsx";
import { useTheme } from "styled-components";

import qrCodeImage from "../../../assets/images/qr_code.png";

import { DisplayCreateInfoStyled, ProcessStepStyled } from "./styled";

const processSteps = [
  {
    id: "start",
    label: "시작",
  },
  {
    id: "mapping",
    label: "제작중",
  },
  {
    id: "finish",
    label: "종료",
  },
];
const DisplayCreateInfo = ({ className, executedCount, totalCount, onEnd }) => {
  const theme = useTheme();
  const currentStep = executedCount >= totalCount ? 'finish' : 'mapping';
  return (
    <DisplayCreateInfoStyled className={clsx("DisplayCreateInfo", className)}>
      <div className="process">
        {processSteps.map((processStep, i) => (
          <ProcessStepStyled
            active={processStep.id === currentStep}
            key={processStep.id}
          >
            <div className="stepSymbol">{i + 1}</div>
            <div className="stepLabel">{processStep.label}</div>
          </ProcessStepStyled>
        ))}
      </div>

      {currentStep === "mapping" && (
        <>
          <div className="status">맵 제작중 ({executedCount}/{totalCount})</div>
          <div className="processbar">
            <ProgressBar completed={executedCount === 0 ? 0 : (executedCount / totalCount * 100).toFixed(1)} height={60} bgColor={theme.colors.primaryColor} />
          </div>
          <div className="message">약 {(totalCount - executedCount) * 2}분뒤 완료</div>
        </>
      )}
      {currentStep === "finish" && (
        <>
          <div className="status">제작 완료</div>
          <div className="qrcode">
            <img src={qrCodeImage} alt="qr코드" />
          </div>
          <div className="action">
            <Button
              block
              size="large"
              type="primary"
              onClick={() => {
                onEnd();
              }}
            >
              돌아가기
            </Button>
          </div>
        </>
      )}
    </DisplayCreateInfoStyled>
  );
};

export default DisplayCreateInfo;
