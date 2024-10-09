#!/usr/bin/env python3
import torch.nn.functional as F
import torch.nn as nn
import torch
import math 

class PositionalEncoding(nn.Module):
    def __init__(self, d_model, dropout=0.1, max_len=5000):
        super(PositionalEncoding, self).__init__()
        self.dropout = nn.Dropout(p=dropout)

        position = torch.arange(max_len).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2) * (-math.log(10000.0) / d_model))
        pe = torch.zeros(max_len, 1, d_model)
        pe[:, 0, 0::2] = torch.sin(position * div_term)
        pe[:, 0, 1::2] = torch.cos(position * div_term)
        self.register_buffer('pe', pe)

    def forward(self, x):
        x = x + self.pe[:x.size(0)]
        return self.dropout(x)
    
class TransformerModel(nn.Module):
    def __init__(self, config):
        super(TransformerModel, self).__init__()
        self.name = "TransformerModel"
        self.config = config

        input_size, d_model, num_layers, output_size, fc_dropout,d_ff_transformer, n_head,head_dropout= (
            config.input_size,
            config.d_model_transformer,
            config.num_layers_transformer,
            config.num_labels,
            config.fc_dropout_transformer,
            config.d_ff_transformer,
            config.transformer_n_head,
            config.head_dropout_transformer
        )
        self.temporal = True
        # self.absenc = AbsolutePositionalEncoding()
        if d_model//n_head != 0:
            d_model = int(d_model/n_head)*n_head

        self.embedding = nn.Linear(input_size, d_model)
        # self.temporal_embed = nn.Linear(1, d_model)
        
        # Positional encoding: Input shape: [batch_size, seq_length, d_model]
        self.positional_encoding = PositionalEncoding(self.config.d_model_transformer, self.config.dropout)

        self.transformer_encoder = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(d_model=d_model, nhead=n_head,
                                    dim_feedforward=d_ff_transformer,activation=F.relu, batch_first=True,
                                    dropout=head_dropout),
            num_layers=num_layers
        )

        fully_connected = []
        current_size = d_model
        while current_size//3 > output_size*2 :
            d_model = current_size//3
            fully_connected.append(nn.Linear(current_size, d_model))
            fully_connected.append(nn.ReLU())
            fully_connected.append(nn.Dropout(fc_dropout))

            current_size = d_model
        
        fully_connected.append(nn.Linear(current_size, output_size))
        self.fully_connected = nn.Sequential(*fully_connected)

        self.fc_sum =  nn.Linear(self.config.sequence_length, 1) 
        # self.Dlinear_sum = DLinear(self.config)

    def forward(self, x, mask=None):
        #x:shape [batch,seq,feture]

        x = self.embedding(x) * math.sqrt(self.config.d_model_transformer)

        x = self.positional_encoding(x)

        x = self.transformer_encoder(x,mask=mask)

        x = self.fully_connected(x) # B, 
        x = x.permute(0,2,1)
        # x = self.Dlinear_sum(x)
        x = self.fc_sum(x)
        x = x.permute(0,2,1) # [batch,1,output=9]
        return x