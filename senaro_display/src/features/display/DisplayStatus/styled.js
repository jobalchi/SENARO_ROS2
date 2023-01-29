import styled from "styled-components";

export const DisplayStatusStyled = styled.div`
  padding: 1rem;
  display: grid;
  width: 100%;
  grid-gap: 1rem;
  grid-template:
    "a a b b" 1fr
    "c c c c" 10rem / 1fr 1fr 1fr 1fr;
`;

export const DisplayStatusCardStyled = styled.div`
  width: 100%;
  border-radius: 1rem;
  background: rgba(0, 0, 0, 0.1);
  padding: 1.5rem;

  &.battery {
    display: flex;
    align-items: center;
    justify-content: center;
    grid-area: a;
    color: ${(props) => props.theme.colors.green};
    .progressCircular {
      width: 70%;
    }
    .percentage {
      font-size: 1.5rem;
      font-weight: bold;
    }
    .CircularProgressbar-path {
      stroke: ${(props) => props.theme.colors.green};
    }
    .CircularProgressbar-trail {
      stroke: #fff;
    }
  }
  &.controller {
    display: flex;
    align-items: center;
    justify-content: center;
    flex-direction: column;
    font-size: 5rem;
    grid-area: b;
    .status {
      font-size: 1.4rem;
      font-weight: bold;
      text-align: center;
    }
  }
  &.sensor {
    grid-area: c;
    display: flex;
    justify-content: space-between;
    gap: 1rem;
  }
`;

export const DisplaySensorStyled = styled.div`
  display: flex;
  align-items: center;
  justify-content: center;
  flex-direction: column;
  background: #ededed;
  width: 100%;
  border-radius: 1rem;
  font-size: 1.2rem;
  font-weight: bold;

  &:before {
    display: block;
    width: 1rem;
    height: 1rem;
    margin-bottom: 1rem;
    border-radius: 50%;
    background: ${(props) => props.theme.colors.green};
    content: "";
  }
`;
