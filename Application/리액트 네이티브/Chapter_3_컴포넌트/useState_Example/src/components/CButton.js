import React from 'react';
import { TouchableOpacity, Text } from 'react-native';

const CButton = () => {
    return(
        <TouchableOpacity
            style={{
                backgroundColor: '#3498db',
                padding: 16,
                margin: 10,
                borderRadius: 8,
            }}
        >
            <Text style={{ color: 'white', fontSize: 24}}>Custom Button</Text>
        </TouchableOpacity>
    );
};

export default CButton;