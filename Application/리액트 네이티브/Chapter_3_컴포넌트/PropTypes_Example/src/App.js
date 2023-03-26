import { View, Text } from 'react-native';
import React from 'react';
import CButton from './components/CButton';

const App = () => {
    return (
        <View
            style={{
                flex: 1,
                backgroundColor: '#fff',
                alignItems: 'center',
                justifyContent: 'center',
            }}
        >
            <Text style={{ fontSize: 30, marginBottom: 10, }}>Props Test</Text>
            <CButton title="Button1" onPress={() => alert('props')} />
            <CButton title="Button2" onPress={() => alert('children')}>
             Children Props
            </CButton>
            <CButton onPress={() => alert('default')} />
        </View>
    );
};

export default App;