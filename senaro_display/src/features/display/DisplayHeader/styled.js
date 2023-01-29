import styled from 'styled-components';

export const DisplayHeaderStyled = styled.div`
  display: flex;
  justify-content: space-between;
  position: absolute;
  z-index: 2;
  left: 0;
  top: 0;
  width: 100%;
  padding: 1rem;
  background: ${props => props.theme.colors.primaryColor};
  color: #fff;

  .logo {
    font-family: 'Bungee', cursive;
    font-size: 1.2rem;
  }
`;
