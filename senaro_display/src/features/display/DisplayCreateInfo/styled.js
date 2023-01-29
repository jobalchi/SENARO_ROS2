import styled, { keyframes } from "styled-components";

export const DisplayCreateInfoStyled = styled.div`
  padding: 4rem;

  .process {
    position: relative;
    display: flex;
    justify-content: space-between;
    margin-bottom: 2rem;

    &:before {
      position: absolute;
      z-index: -1;
      top: 5vw;
      left: 0;
      width: 100%;
      height: 1px;
      background: #ccc;
      content '';
    }
  }
  .status {
    margin-bottom: 2rem;
    font-size: 2.5rem;
    text-align: center;
  }

  .message {
    display: flex;
    justify-content: center;
    margin: 2rem 0;
    font-size: 2rem;
    color: #999;
    font-weight: 100;
  }

  .processbar span {
    font-size: 1.2rem !important;
  }

  .qrcode {
    display: flex;
    justify-content: center;
    margin-bottom: 2rem;
    img {
      width: 40%;
    }
  }
`;

export const ProcessStepStyled = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  font-size: 2rem;
  .stepSymbol {
    position: relative;
    display: flex;
    align-items: center;
    justify-content: center;
    width: 10vw;
    aspect-ratio: 1 / 1;
    border-radius: 50%;
    background: #ededed;
    margin-bottom: 0.5rem;
    transition: 300ms ease;

    ${(props) =>
      props.active &&
      `
      background: ${props.theme.colors.primaryColor};
      font-weight: bold;
      color: #fff;

      &:before {
        position: absolute;
        z-index: -1;
        left: 0;
        top: 0;
        width: 100%;
        height: 100%;
        animation: bounce 1s infinite;
        background: ${props.theme.colors.primaryColor};
        border-radius: 50%;
        content: '';
      }
    `}
  }

  ${(props) =>
    props.active
      ? `
      font-weight: bold;
  `
      : `
    color: #666;
  `}
`;
