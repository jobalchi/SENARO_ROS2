import styled from 'styled-components';

export const ProjectsViewPageStyled = styled.div`
  .projectMenu {
    position: fixed;
    top: 1rem;
    right: 1rem;
    display: flex;
    flex-direction: column;
    gap: 0.5rem;
  }

  .viewer {
    position: fixed;
    top: 0;
    left: 0;
    width: 100vw;
    height: 100vh;
  }
`;

export const ConfigButtonStyled = styled.button<{
  active?: boolean;
}>`
  width: 40px;
  height: 40px;
  border-radius: 0.15rem;
  background: #fff;
  border: none;
  color: #000;
  font-size: 1.15rem;
  transition: 300ms ease;
  ${props =>
    props.active &&
    `
    background: ${props.theme.colors.primaryColor};
    color: #fff;
  `}
`;

export const MarkerInfoStyled = styled.div<{
  active: boolean;
}>`
  position: fixed;
  left: 0;
  bottom: 0;
  width: 100%;
  min-height: 15vh;
  max-height: 25vh;
  overflow: auto;
  background: #fff;
  padding: 1rem;
  border-top-left-radius: 1rem;
  border-top-right-radius: 1rem;
  visibility: hidden;
  opacity: 0;
  transform: translateY(5rem);
  transition: 300ms ease;

  .title {
    font-size: 1.4rem;
    font-weight: bold;
    margin-bottom: 0.5rem;
  }
  .content {
    font-size: 1rem;
    color: #333;
  }
  .closeButton {
    position: absolute;
    top: 1rem;
    right: 1rem;
    background: none;
    border: none;
    font-size: 1.5rem;
    color: #000;
  }

  ${props =>
    props.active
      ? `
    opacity: 1;
    visibility: visible;
    transform: translateY(0);
  `
      : `
    .title, .content {
      display: none;
    }
  `}
`;
