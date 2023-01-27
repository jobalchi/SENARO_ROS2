import styled, { createGlobalStyle } from 'styled-components';

export const DisplayPageStyled = styled.div`
  position: relativie;
  width: 100vw;
  height: 100vh;
  overflow: hidden;

  .action {
    padding: 1rem;
    .ant-btn {
      widith: 100%;
      border-radius: 1rem;
      height: 6rem;
      font-size: 1.45rem;
      font-weight: bold;
    }
  }
`;

export const DisplayGlobalStyled = createGlobalStyle`
  body, html {
    font-size: 3vw;
  }
  html {
    touch-action:none;
  }
`;
